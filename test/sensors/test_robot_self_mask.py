import struct

import numpy as np
import pytest

cv2 = pytest.importorskip('cv2')

from stretch4_rtabmap.sensors.robot_self_mask import (  # noqa: E402
    apply_mask,
    blend,
    box_corners,
    build_mask,
    classify_features,
    clip_box_to_near_plane,
    feather_alpha,
    fill_image,
    FILL_MODES,
    joints_moved,
    link_boxes_from_urdf,
    load_stl_vertices,
    moving_joints_from_urdf,
    NEAR_PLANE,
    project_hull,
)

# fx = fy = 100, cx = 50, cy = 40 on a 100x80 image.
P_LEFT = np.array([[100.0, 0.0, 50.0, 0.0], [0.0, 100.0, 40.0, 0.0], [0.0, 0.0, 1.0, 0.0]])


def write_binary_stl(path, triangles):
    with open(path, 'wb') as f:
        f.write(b'\0' * 80)
        f.write(struct.pack('<I', len(triangles)))
        for tri in triangles:
            f.write(struct.pack('<3f', 0.0, 0.0, 1.0))
            for vertex in tri:
                f.write(struct.pack('<3f', *vertex))
            f.write(b'\0\0')


def test_load_binary_stl(tmp_path):
    path = tmp_path / 'tri.stl'
    write_binary_stl(path, [[(0, 0, 0), (1, 0, 0), (0, 2, 3)]])
    verts = load_stl_vertices(str(path))
    assert verts.shape == (3, 3)
    assert np.allclose(verts.max(axis=0), [1, 2, 3])


def test_urdf_boxes_apply_origin_scale_and_padding(tmp_path):
    path = tmp_path / 'part.stl'
    write_binary_stl(path, [[(0, 0, 0), (1, 0, 0), (0, 1, 1)]])
    urdf = f"""<robot name="r">
      <link name="a"><collision><origin xyz="1 0 0" rpy="0 0 0"/>
        <geometry><mesh filename="file://{path}" scale="2 2 2"/></geometry></collision></link>
      <link name="b"><collision><geometry><box size="0.2 0.4 0.6"/></geometry></collision></link>
      <link name="skipped"><collision><geometry><box size="1 1 1"/></geometry></collision></link>
      <link name="empty"/>
    </robot>"""
    boxes = link_boxes_from_urdf(urdf, exclude=['skipped'], padding=0.1)
    assert set(boxes) == {'a', 'b'}
    assert np.allclose(boxes['a'][0], [0.9, -0.1, -0.1])
    assert np.allclose(boxes['a'][1], [3.1, 2.1, 2.1])
    assert np.allclose(boxes['b'][1], [0.2, 0.3, 0.4])


def test_box_in_front_projects_to_expected_pixels():
    corners = box_corners(np.array([-0.1, -0.1, 1.0]), np.array([0.1, 0.1, 1.2]))
    polygon = project_hull(clip_box_to_near_plane(corners), P_LEFT, 100, 80).reshape(-1, 2)
    # The near face (z = 1.0) spans +/- 10 px around the principal point.
    assert polygon[:, 0].min() == 40 and polygon[:, 0].max() == 60
    assert polygon[:, 1].min() == 30 and polygon[:, 1].max() == 50


def test_box_behind_camera_is_dropped():
    corners = box_corners(np.array([-0.1, -0.1, -1.2]), np.array([0.1, 0.1, -1.0]))
    assert project_hull(clip_box_to_near_plane(corners), P_LEFT, 100, 80) is None


def test_box_crossing_near_plane_is_clipped_and_kept_in_image():
    corners = box_corners(np.array([-0.05, 0.0, -0.5]), np.array([0.05, 0.3, 0.5]))
    clipped = clip_box_to_near_plane(corners)
    assert clipped[:, 2].min() >= NEAR_PLANE - 1e-9
    polygon = project_hull(clipped, P_LEFT, 100, 80).reshape(-1, 2)
    # Below the camera and reaching the bottom edge, clipped to the image.
    assert polygon[:, 1].max() == 80
    assert polygon.min() >= 0 and polygon[:, 0].max() <= 100


def test_right_camera_baseline_shifts_projection_left():
    p_right = P_LEFT.copy()
    p_right[0, 3] = -100.0 * 0.15  # -fx * baseline
    corners = box_corners(np.array([-0.1, -0.1, 1.0]), np.array([0.1, 0.1, 1.2]))
    clipped = clip_box_to_near_plane(corners)
    left = project_hull(clipped, P_LEFT, 100, 80).reshape(-1, 2)
    right = project_hull(clipped, p_right, 100, 80).reshape(-1, 2)
    assert right[:, 0].max() < left[:, 0].max()


def textured_image(seed=0):
    rng = np.random.default_rng(seed)
    image = cv2.GaussianBlur((rng.random((240, 320)) * 255).astype(np.uint8), (0, 0), 2)
    return cv2.normalize(image, None, 0, 255, cv2.NORM_MINMAX)


ARM = [np.array([[80, 120], [200, 100], [220, 170], [90, 190]], dtype=np.int32)]


def test_build_mask_merges_dilates_and_adds_static():
    static = np.zeros((240, 320), dtype=bool)
    static[0:5, 0:5] = True
    plain = build_mask(ARM, 320, 240)
    dilated = build_mask(ARM, 320, 240, dilate_px=5, static=static)
    assert dilated[2, 2] == 255 and plain[2, 2] == 0
    assert (dilated >= plain).all() and (dilated > 0).sum() > (plain > 0).sum()


@pytest.mark.parametrize('mode', FILL_MODES)
def test_every_mode_replaces_everything_inside_the_mask(mode):
    image = textured_image()
    mask = build_mask(ARM, 320, 240, dilate_px=4)
    out = apply_mask(image, mask, mode, fill_value=0, blur_ksize=31, feather_px=8)
    assert out.dtype == image.dtype and out.shape == image.shape
    inside = mask > 0
    # No texture survives inside: the fill is far smoother than the original there.
    assert out[inside].std() < 0.5 * image[inside].std()
    far = cv2.dilate(mask, np.ones((41, 41), np.uint8)) == 0
    assert np.array_equal(out[far], image[far])


def test_color_image_keeps_channels():
    image = cv2.merge([textured_image(s) for s in range(3)])
    out = apply_mask(image, build_mask(ARM, 320, 240), 'blur', blur_ksize=31, feather_px=5)
    assert out.shape == image.shape


def test_soft_fill_has_fewer_corners_on_the_mask_edge_than_a_hard_fill():
    image = textured_image()
    mask = build_mask(ARM, 320, 240, dilate_px=6)
    hard = apply_mask(image, build_mask(ARM, 320, 240), 'constant')
    soft = apply_mask(image, mask, 'blur', blur_ksize=31, feather_px=10)
    hard_near, _ = classify_features(hard, mask, 4)
    soft_near, _ = classify_features(soft, mask, 4)
    assert len(soft_near) < len(hard_near) / 2


def test_moving_joints_are_prismatic_and_revolute_only():
    urdf = """<robot name="r">
      <joint name="lift_joint" type="prismatic"/><joint name="wrist_yaw_joint" type="revolute"/>
      <joint name="wheel_0_joint" type="continuous"/><joint name="mount" type="fixed"/>
    </robot>"""
    assert moving_joints_from_urdf(urdf) == {'lift_joint', 'wrist_yaw_joint'}


def test_joints_moved():
    names = {'lift_joint', 'wrist_yaw_joint'}
    ref = {'lift_joint': 0.5, 'wrist_yaw_joint': 1.0, 'wheel_0_joint': 0.0}
    assert joints_moved(None, ref, names, 0.005)                       # no mask yet
    assert not joints_moved(ref, dict(ref, wheel_0_joint=12.3), names, 0.005)  # wheels ignored
    assert not joints_moved(ref, dict(ref, lift_joint=0.503), names, 0.005)    # within tolerance
    assert joints_moved(ref, dict(ref, lift_joint=0.51), names, 0.005)
    assert not joints_moved(ref, {'wheel_0_joint': 1.0}, names, 0.005)         # not published


def test_cached_alpha_gives_the_same_image():
    rng = np.random.default_rng(1)
    image = (rng.random((80, 100)) * 255).astype(np.uint8)
    mask = np.zeros((80, 100), np.uint8)
    mask[20:50, 30:70] = 255
    for feather in (0, 8):
        expected = apply_mask(image, mask, 'blur', blur_ksize=21, feather_px=feather)
        cached = blend(image, fill_image(image, mask, 'blur', blur_ksize=21), mask,
                       feather_alpha(mask, feather))
        assert np.array_equal(expected, cached)
