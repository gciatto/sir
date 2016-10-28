from pathlib import Path
import glob

resources_path = Path(__file__).parent


def paths(name='*', ext='*'):
    pattern = name
    if ext is not None:
        pattern += ext
    pattern = str(resources_path.joinpath(pattern))
    g = glob.glob(pattern)
    if len(g) == 1:
        return g[0]
    else:
        return g
