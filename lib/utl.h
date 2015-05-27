#ifndef UTL_H_
#define UTL_H_

namespace hll
{
	/**
	 * 1-D Gaussian sampler
	 * @param mean, sigma
	 * @return float
	 */
	float sample(float mean, float sigma);
	
	/**
	 * contrain value to its range
	 * @param input value, maximum vlue, minimum value
	 * @return float
	 */
	float constrain(float value, float vmax, float vmin);
} /* end of namespace */
#endif /* SimulatorApp_H_ */
