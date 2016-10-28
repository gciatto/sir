from pathlib import Path
import glob

resources_path = Path(__file__).parent


def paths(name='*', ext='*'):
    pattern = name
    if ext is not None:
        pattern += ext
    pattern = str(resources_path.joinpath(pattern))
    paths_list = glob.glob(pattern)
    if len(paths_list) == 1:
        return paths_list[0]
    else:
        return paths_list
