follow the coding standard files in folder `assets/manuals/standard`
dont use any `printf` to display any information ,use `src.support.log` module instead.

keep coding style elegant and consistent with the rest of the project
only use path setting in the code in main.py.

you must judge where to put the file or the folder according to the standard or README.md when you will build newly,and you cant build those randomly.

before you execute the plan, you must tell me the plan in detail.you can execute the plan only after i give you the permission.
after generate code, you must preview the code according the standards in folder `assets/manuals/standard`.

make the following code as the standard relative lib import code in all python files under paragraph `if name == "__main__":`   
```
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).parents[2]
sys.path.insert(0, str(PROJECT_ROOT))
```
