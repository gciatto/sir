from pathlib import Path
import glob

resources_path = Path(__file__).parent


def paths(parent='*', name='*', ext='*'):
    pattern = name
    if ext is not None:
        pattern += ext
    pattern = str(resources_path.joinpath(parent, pattern))
    paths_list = glob.glob(pattern, recursive=True)
    if len(paths_list) == 1:
        return paths_list[0]
    else:
        return paths_list


def arenas(name='*', ext='*'):
    return paths('arenas', name, ext)


def layouts(name='*', ext='*'):
    return paths('layouts', name, ext)