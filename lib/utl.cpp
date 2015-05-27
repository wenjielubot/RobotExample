#include <random>
#include <cmath>
#include <utility>
#include "utl.h"

namespace hll
{
	/**
	 * 1-D Gaussian sampler
	 * @param mean, sigma
	 * @return float
	 */	
	float sample (float mean, float sigma)
	{
	    srand (time(NULL));
	    std::random_device rdd;
	    std::mt19937 gn(rdd());
	
	    std::normal_distribution<> nd(mean,sigma);
	
	    return nd(gn);
	}

	/**
	 * contrain value to its range
	 * @param input value, maximum vlue, minimum value
	 * @return float
	 */	
	float constrain (float value, float vmax, float vmin)
	{
	    //force value to its range
	    if (value > vmax) return vmax;
	
	    if (value < vmin ) return vmin;
	
	    return value;
	}

}
