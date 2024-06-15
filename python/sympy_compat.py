# -*- coding: utf-8 -*-
# Copyright (C) 2006-2011 Rosen Diankov <rosen.diankov@gmail.com>
#               2015-2016 Bahram Maleki-Fard <maleki-fard@kbsg.rwth-aachen.de>
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License. 
"""Backward compatibility for older sympy versions (>= 0.7.1)
   Basically a lot of monkey-patching.

   Make sure to load the file after importing sympy.*
   Also make sure to use execfile(..) to do so, in order for the patches
   to be applied.

   This file has been created to work primarily with ikfast, but should
   be applicable to other files that use sympy as well.
   However, this requires a "from sympy import ..." prior to loading the
   file.
"""

from sympy import __version__ as sympy_version

# core/power.py Pow
def Pow_eval_subs(self, old, new):
    if self == old: 
        return new
    
    if old.func is self.func and self.base == old.base:
        coeff1, terms1 = self.exp.as_coeff_mul()
        coeff2, terms2 = old.exp.as_coeff_mul()
        if terms1==terms2:
#             pow = coeff1/coeff2
#             if pow.is_Integer or self.base.is_commutative:
#                 return Pow(new, pow) # (x**(2*y)).subs(x**(3*y),z) -> z**(2/3)
            # only divide if coeff2 is a divisor of coeff1
            if coeff1.is_integer and coeff2.is_integer and (coeff1/coeff2).is_integer:
                return new ** (coeff1/coeff2) # (x**(2*y)).subs(x**(3*y),z) -> z**(2/3*y)
            
            if old.func is C.exp:
                coeff1, terms1 = old.args[0].as_coeff_mul()
                coeff2, terms2 = (self.exp*C.log(self.base)).as_coeff_mul()
                if terms1==terms2:
                    # only divide if coeff2 is a divisor of coeff1
                    if coeff1.is_integer and coeff2.is_integer and (coeff1/coeff2).is_integer:
                        return new ** (coeff1/coeff2) # (x**(2*y)).subs(exp(3*y*log(x)),z) -> z**(2/3*y)
                    
    return Pow(self.base._eval_subs(old, new), self.exp._eval_subs(old, new))


if sympy_version < '0.7.2':
    # matrices/matrices.py (matrices/dense.py)
    # API-changes in zeros() and ones()
    from sympy import zeros as _zeros, ones as _ones
    zeros = lambda args: _zeros(*args)
    ones = lambda args: _ones(*args)

    # polys/monomialtools.py
    # API-changes in Monomial.__init__
    from sympy import Monomial as _Monomial
    _Monomial_init = _Monomial.__init__
    Monomial.__init__ = lambda self, m, gens=None: _Monomial_init(self, *m)


if sympy_version < '0.7.3':
    # core/power.py
    # Workaround that has been fixed in sympy >= 0.7.3
    power.Pow._eval_subs = Pow_eval_subs

if sympy_version < '0.7.4':
    # simplify/simplify.py
    # trigsimp was already expanded (not simplified)
    from sympy.simplify import trigsimp as _trigsimp
    trigsimp_expanded = lambda expr, **opts: _trigsimp(expr, **opts)

