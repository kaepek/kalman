import pathlib
import sys

root = pathlib.Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(root))
sys.path.insert(0, str(root / "tests" / "reference"))
