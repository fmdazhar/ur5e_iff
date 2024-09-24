"""
Numba utils.
"""
import numba

import macros  # Relative import to access macros


def jit_decorator(func):
    if macros.ENABLE_NUMBA:
        return numba.jit(nopython=True, cache=macros.CACHE_NUMBA)(func)
    return func
