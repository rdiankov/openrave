// http://people.sc.fsu.edu/~jburkardt/cpp_src/halton/halton.html
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
#ifndef SAMPLER_HALTON
#define SAMPLER_HALTON

#include <openrave/openrave.h>
using namespace OpenRAVE;
using namespace std;

class HaltonSampler : public SpaceSamplerBase
{
public:
    HaltonSampler(EnvironmentBasePtr penv, std::istream& sinput) : SpaceSamplerBase(penv)
    {
        __description = ":Interface Author: John Burkardt\n\n\
References:\n\n\
1. John Halton, On the efficiency of certain quasi-random sequences of points in evaluating multi-dimensional integrals, Numerische Mathematik, Volume 2, 1960, pages 84-90.\n\n\
2. John Halton, GB Smith, Algorithm 247: Radical-Inverse Quasi-Random Point Sequence, Communications of the ACM, Volume 7, 1964, pages 701-702.\n\n\
3. Ladislav Kocis, William Whiten, Computational Investigations of Low-Discrepancy Sequences, ACM Transactions on Mathematical Software, Volume 23, Number 2, 1997, pages 266-294.\n\n\
";
        halton_BASE = NULL;
        halton_LEAP = NULL;
        halton_DIM_NUM = -1;
        halton_SEED = NULL;
        halton_STEP = -1;
        SetSpaceDOF(1);
        SetSeed(0);
        halton_step_set (1);
    }

    void SetSeed(uint32_t seed) {
        vector<int> vseed(halton_dim_num_get(),0);
        halton_seed_set ( &vseed[0] );
    }

    void SetSpaceDOF(int dof) {
        BOOST_ASSERT(dof > 0);
        halton_dim_num_set ( dof );
    }
    int GetDOF() const {
        return halton_dim_num_get();
    }
    int GetNumberOfValues() const {
        return halton_dim_num_get();
    }
    bool Supports(SampleDataType type) const {
        return type==SDT_Real;
    }

    void GetLimits(std::vector<dReal>& vLowerLimit, std::vector<dReal>& vUpperLimit) const
    {
        vLowerLimit.resize(halton_dim_num_get());
        vUpperLimit.resize(halton_dim_num_get());
        for(int i = 0; i < halton_dim_num_get(); ++i) {
            vLowerLimit[i] = 0;
            vUpperLimit[i] = 1;
        }
    }

    void SampleSequence(std::vector<dReal>& samples, size_t num=1,IntervalType interval=IT_Closed)
    {
        samples.resize(halton_dim_num_get()*num);
        halton_sequence(num,&samples[0]);
    }

    dReal SampleSequenceOneReal(IntervalType interval=IT_Closed)
    {
        OPENRAVE_ASSERT_OP_FORMAT0(GetDOF(),==,1,"sample can only be 1 dof", ORE_InvalidState);
        dReal f=0;
        halton_sequence(1,&f);
        return f;
    }

protected:
    dReal arc_cosine ( dReal c );
    dReal atan4 ( dReal y, dReal x );
    char digit_to_ch ( int i );
    int get_seed ( void );
    void halham_write ( int dim_num, int n, int step, int seed[], int leap[], int base[], dReal r[], char *file_out_name );
    void halton ( dReal r[] );
    int *halton_base_get ( void ) const;
    void halton_base_set ( int base[] );
    int *halton_leap_get ( void ) const;
    void halton_leap_set ( int leap[] );
    int halton_dim_num_get ( void ) const;
    void halton_dim_num_set ( int dim_num );
    int *halton_seed_get ( void ) const;
    void halton_seed_set ( int seed[] );
    void halton_sequence ( int n, dReal r[] );
    int halton_step_get ( void );
    void halton_step_set ( int step );
    int i4_log_10 ( int i );
    int i4_min ( int i1, int i2 );
    void i4_to_halton ( int dim_num, int step, int seed[], int leap[], int base[], dReal r[] );
    void i4_to_halton_sequence ( int dim_num, int n, int step, int seed[], int leap[], int base[], dReal r[] );
    char *i4_to_s ( int i );
    int prime ( int n );
    dReal r8_epsilon ( void );
    dReal r8vec_dot_product ( int n, dReal *r1, dReal *r2 );
    dReal r8vec_norm_l2 ( int n, dReal a[] );
    int s_len_trim ( char *s );

    //
    //  These variables are accessible to the user via calls to routines.
    //
    int *halton_BASE;
    int *halton_LEAP;
    int halton_DIM_NUM;
    int *halton_SEED;
    int halton_STEP;
};

#endif
