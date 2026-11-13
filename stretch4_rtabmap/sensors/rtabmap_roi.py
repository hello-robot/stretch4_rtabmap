"""
RTAB-Map image ROI helpers.

RTAB-Map expresses its region of interest as four ratios, "left right top bottom", giving the
fraction of each edge that is CROPPED AWAY before feature extraction.

"""

# Fractions cropped from each edge, in RTAB-Map's order.
EDGE_NAMES = ('left', 'right', 'top', 'bottom')

# RoiRatios can only crop full-width/full-height edge strips.
# No crop: the pitched-up stereo view (stretch_core stereo_pitch_deg) no longer sees the base.
MAST_ROI_RATIOS = '0.0 0.0 0.0 0.0'

def parse_roi_ratios(value):
    """Parse a RoiRatios string, returning None if RTAB-Map would reject it."""
    parts = value.split()
    if len(parts) != 4:
        return None
    try:
        return tuple(float(p) for p in parts)
    except ValueError:
        return None


def ignored_edges(ratios):
    """Which non-zero ratios util2d::computeRoi's bounds guard drops on the floor."""
    opposites = (ratios[1], ratios[0], ratios[3], ratios[2])
    return [name for name, ratio, opposite in zip(EDGE_NAMES, ratios, opposites)
            if ratio > 0 and ratio >= 1.0 - opposite]


def validate_roi_ratios(value, argument='mast_roi_ratios'):
    """Raise on any ROI value RTAB-Map would reject or silently ignore."""
    ratios = parse_roi_ratios(value)
    if ratios is None:
        raise RuntimeError(
            f'{argument} must be 4 space-separated numbers "left right top bottom", '
            f'got "{value}".')

    for name, ratio in zip(EDGE_NAMES, ratios):
        if ratio < 0.0:
            raise RuntimeError(f'{argument} {name}={ratio} is negative, in "{value}".')

    skipped = ignored_edges(ratios)
    if skipped:
        raise RuntimeError(
            f'{argument} would be silently ignored by RTAB-Map for: {", ".join(skipped)}. Each '
            f'non-zero ratio must be less than 1 minus its opposite edge. Value was "{value}".')

    return ratios
