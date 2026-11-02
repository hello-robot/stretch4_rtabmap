"""RTAB-Map parameter files (config/rtabmap_params_*.yaml): loading, and what differs by mode."""

import os

import yaml


def load_params_file(path, seen=()):
    """Load a params file, applying its 'inherit:' base first so the child overrides it."""
    real = os.path.realpath(path)
    if real in seen:
        raise RuntimeError(f'inherit loop in rtabmap params files at {path}')

    with open(path) as f:
        doc = yaml.safe_load(f) or {}
    unknown = set(doc) - {'rtabmap_parameters', 'inherit'}
    if unknown:
        raise RuntimeError(f'{path}: unknown top-level keys {sorted(unknown)}')
    if 'rtabmap_parameters' not in doc:
        raise RuntimeError(f'{path}: missing top-level "rtabmap_parameters:" key')

    params, chain = {}, []
    if doc.get('inherit'):
        base = os.path.join(os.path.dirname(os.path.abspath(path)), str(doc['inherit']))
        params, chain = load_params_file(base, seen + (real,))
    params.update(doc['rtabmap_parameters'] or {})
    return params, chain + [path]


def render(value):
    """Return a parameter value as the string RTAB-Map takes."""
    return ('true' if value else 'false') if isinstance(value, bool) else str(value)


def same(a, b):
    """Return whether two rendered values mean the same: "0.10" and "0.1" do."""
    try:
        return float(a) == float(b)
    except ValueError:
        return a.lower() == b.lower()


def mode_params(mapping_file, localization_file):
    """
    Return ({key: mapping value}, {key: localization value}) for the keys that differ.

    A key set in only one file must be set in both: the other file has no value to switch back to.
    """
    mapping, _ = load_params_file(mapping_file)
    localization, _ = load_params_file(localization_file)
    missing = sorted(set(mapping) ^ set(localization))
    if missing:
        raise RuntimeError(f'set in only one of {mapping_file} and {localization_file}: '
                           f'{missing}')
    keys = [k for k in mapping if not same(render(mapping[k]), render(localization[k]))]
    return ({k: render(mapping[k]) for k in keys}, {k: render(localization[k]) for k in keys})
