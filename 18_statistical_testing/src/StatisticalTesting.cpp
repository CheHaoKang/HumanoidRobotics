#include <statistical_testing/StatisticalTesting.h>
#include <boost/math/distributions/normal.hpp>
#include <cmath>
using boost::math::normal; // typedef provides default type is double.
namespace statistical_testing {

/**
 * \brief Calculates the mean of a vector of numbers.
 * \param[in] values The vector of numbers.
 * \return The mean (average) of the numbers.
 */
double StatisticalTesting::mean(const std::vector<double>& values) {
	double mean = 0.0;
	// TODO calculate the mean.
	std::vector<double>::const_iterator it ;
	double total = 0;
	int count = values.size();
	for(it=values.begin();it!=values.end();it++)
	{
		total +=*it;
	}
	mean = total / count;
	return mean;
}

/**
 * \brief Calculates the Z score of a vector of numbers.
 * \param[in] values The vector of numbers.
 * \param[in] distributionMean The mean of the entire population.
 * \param[in] distributionStandardDeviation the standard deviation of the entire population.
 * \return The Z score.
 */
double StatisticalTesting::zScore(const std::vector<double>& values, const double& distributionMean, const double& distributionStandardDeviation) {
	double z = 0.0;
	// TODO calculate the Z score of the values.
	double Mean =mean(values);
	z = (Mean - distributionMean) / (distributionStandardDeviation/sqrt(values.size()));
	return z;
}

/**
 * \brief Executes a Z test for the "German students test" example on the slides.
 */
void StatisticalTesting::germanStudentsTest() {
	const double testResults[] = {
			97, 77, 100, 99, 100, 75, 76, 95, 96, 90,
			96, 70, 71, 98, 97, 97, 67, 100, 97, 100,
			92, 130, 100, 100, 95, 100, 92, 94, 89,
			89, 82, 65, 100, 98, 85, 100, 93, 87, 100,
			97, 73, 100, 93, 110, 95, 110, 79, 92, 96,
			100, 87, 92, 110, 110, 100
	};
	const double distributionMean = 100;
	const double distributionStandardDeviation = 12;
	const double confidenceLevel = 0.95;

	TestCase test;
	Hypothesis a("Bonn students are better than other students", GREATER);
	Hypothesis b("Bonn students are worse than other students", LESS);
	Hypothesis c("Bonn students are at least as good as other students", AT_LEAST);
	Hypothesis d("Bonn students are at most as good as other students", AT_MOST);
	Hypothesis e("Bonn students are as good as other students", EQUAL);
	Hypothesis f("Bonn students perform differently from other students", DIFFERENT);

	/* TODO: Select a hypothesis and a null hypothesis from the above choices a-f and
	 * call test.setHypothesis(...) and test.setNullHypothesis(...) with the chosen
	 * variable.
	 */
    test.setHypothesis(b);
    test.setNullHypothesis(c);

	oneSampleZTest(arrayToVector(testResults), distributionMean, distributionStandardDeviation, confidenceLevel, test);
}


/**
 * \brief Runs a one-sample Z test on a vector of numbers.
 * \param[in] values The vector of numbers.
 * \param[in] distributionMean The mean of the entire population.
 * \param[in] distributionStandardDeviation the standard deviation of the entire population.
 * \param[in] The confidence level (commonly used values: 0.95, 0.999)
 * \param[in] testCase The test case containing the hypothesis and null hypothesis chosen in germanStudentsTest()
 */
void StatisticalTesting::oneSampleZTest(const std::vector<double>& values,
		const double& distributionMean, const double& distributionStandardDeviation,
		const double& confidenceLevel, const TestCase& testCase) {

	/**
	 * TODO: Execute the Z test for the given vector of numbers and either
	 * reject the null hypothesis or state that you cannot reject the null hypothesis.
	 */

	/*
	 * Available methods:
	 * - lookupZTable(double Z): returns the cumulative density function
	 *       of the standard normal distribution at Z (see slide 23 for examples).
	 * - testCase.getHypothesis(): returns the hypothesis (see germanStudentsTest() above)
	 * - testCase.getNullHypothesis(): returns the null hypothesis (see germanStudentsTest() above)
	 *
	 * For both the hypothesis and the null hypothesis, the following methods are available:
	 * - hypothesis.getDirection(): Returns one element from the following enumeration:
	 *       LESS, GREATER, AT_LEAST, AT_MOST, EQUAL, or DIFFERENT.
	 * - hypothesis.reject(): Rejects the hypothesis.
	 * - hypothesis.cannotReject(): States that we cannot reject the hypothesis based on the data.
	 */
	double zscore = zScore(values,distributionMean,distributionStandardDeviation);
	double zlookup = lookupZTable(zscore);
	const Hypothesis& h1 = testCase.getHypothesis();
	const Hypothesis& h0 = testCase.getNullHypothesis();
	double zconfidence = 0;
	normal s;
	/* If greater or lesser the area of significance is on one side of curve. so we use quantile with same
	 * confidence level. Else we can divide alpha by 2 for two sided test
	 *
	 */
	if (h0.getDirection()==AT_MOST) 	{
		zconfidence = confidenceLevel;
		if (zlookup>zconfidence)
			h0.reject();
			else 		{
				h0.cannotReject();
			}
	}
	else {
		if(h0.getDirection()==AT_LEAST)
		{
			zconfidence = (1-confidenceLevel);
			if (zlookup<zconfidence)
				h0.reject();
				else {
					h0.cannotReject();
				}
		}
		else {
			double left_boundry = (1-confidenceLevel);
			double right_boundry = confidenceLevel;
						if((zlookup<left_boundry)||(zlookup>right_boundry))
							h0.reject();
							else 	{
								h0.cannotReject();
							}
		}
	}
}
/**
 * \brief Calculates the sample standard deviation of a vector of numbers.
 * \param[in] values The vector of numbers.
 * \return The sample standard deviation of the numbers.
 */
double StatisticalTesting::standardDeviation(const std::vector<double>& values) {
	double sigma = 0.0;
	/* TODO Calculate the standard deviation of the values.
	 * You can use the method mean() from above.
	 *
	 * Caution: Make sure to calculate the *sample* standard deviation,
	 * not the *distribution/population* standard deviation.
	 */
	double Mean = mean(values);
	double sigmasquare =0;
	std::vector<double>::const_iterator it;
	for (it=values.begin();it!=values.end();it++) 	{
		sigmasquare += pow(Mean-(*it),2);
	}
	sigmasquare /= (values.size()-1);
	sigma= sqrt(sigmasquare);
	return sigma;
}


/**
 * \brief Calculates the t value of a vector of numbers.
 * \param[in] values The vector of numbers.
 * \param[in] distributionMean The mean of the entire population.
 * \return The t value.
 */
double StatisticalTesting::tValue(const std::vector<double>& values, const double& distributionMean) {
	double t = 0.0;
	// TODO Calculate the t value. Estimate the standard deviation from the sample.
	double samplesigma =standardDeviation(values) ;
	double Mean = mean(values);
	t= (Mean - distributionMean)/(samplesigma /sqrt(values.size()));
	return t;
}


/**
 * \brief Runs a one-sample t test on a vector of numbers.
 * \param[in] values The vector of numbers.
 * \param[in] distributionMean The mean of the entire population.
 * \param[in] The confidence level (commonly used values: 0.95, 0.999)
 * \param[in] testCase The test case containing the hypothesis and null hypothesis chosen in germanStudentsTest()
 */
void StatisticalTesting::oneSampleTTest(const std::vector<double>& values, const double& distributionMean,
		const double& confidenceLevel, const TestCase& testCase) {
	/**
	 * TODO: Execute the one sample T test for the given vector of numbers and either
	 * reject the null hypothesis or state that you cannot reject the null hypothesis.
	 */

	/*
	 * Available methods:
	 * - lookupTTable(size_t degreeOfFreedom, double confidenceLevel, TestType testType):
	 *       returns the quantile function of the student's t distribution (see slide
	 *       of the standard normal distribution at Z (see slide 29 for examples).
	 *       testType can be either ONE_SIDED or TWO_SIDED.
	 * - testCase.getHypothesis(): returns the hypothesis (see germanStudentsTest() above)
	 * - testCase.getNullHypothesis(): returns the null hypothesis (see germanStudentsTest() above)
	 *
	 * For both the hypothesis and the null hypothesis, the following methods are available:
	 * - hypothesis.getDirection(): Returns one element from the following enumeration:
	 *       LESS, GREATER, AT_LEAST, AT_MOST, EQUAL, or DIFFERENT.
	 * - hypothesis.reject(): Rejects the hypothesis.
	 * - hypothesis.cannotReject(): States that we cannot reject the hypothesis based on the data.
	 */
	size_t dof = values.size() -1;
	double tscore = tValue(values,distributionMean);
	double tlookup ;
	const Hypothesis& h1 = testCase.getHypothesis();
	const Hypothesis& h0 = testCase.getNullHypothesis();
		/* If greater or lesser the area of significance is on one side of curve. so we use quantile with same
		 * confidence level. Else we can divide alpha by 2 for two sided test
		 *
		 */
		if (h0.getDirection()==AT_MOST) 	{
			tlookup = lookupTTable(dof,confidenceLevel,StatisticalTesting::ONE_SIDED);
			if (tscore >tlookup )
				h0.reject();
				else 		{
					h0.cannotReject();
				}
		}
		else {
			if(h0.getDirection()==AT_LEAST)
			{
				tlookup = lookupTTable(dof,(1-confidenceLevel),StatisticalTesting::ONE_SIDED);
				if (tscore < tlookup )
					h0.reject();
					else {
						h0.cannotReject();
					}
			}
			else {
				double right_boundry = lookupTTable(dof,confidenceLevel,StatisticalTesting::ONE_SIDED);
				double left_boundry = lookupTTable(dof,(1-confidenceLevel),StatisticalTesting::ONE_SIDED);
							if((tscore<left_boundry)||(tscore>right_boundry))
								h0.reject();
								else 	{
									h0.cannotReject();
								}
			}
		}


}

/**
 * \brief Executes a one sample T test for the "Cars" example on the slides.
 */
void StatisticalTesting::cars() {
	const double prices[] = {11492.70, 23848.70, 15096.80, 27376.10, 15576.50};
	const double distributionMean = 12000;
	const double confidenceLevel = 0.95;

	TestCase test;
	Hypothesis a("The cars are more expensive than in the rest of the city", GREATER);
	Hypothesis b("The cars are cheaper than in the rest of the city", LESS);
	Hypothesis c("The cars are at least as expensive as in the rest of the city", AT_LEAST);
	Hypothesis d("The cars are at most as expensive as in the rest of the city", AT_MOST);
	Hypothesis e("The cars are as expensive as in the rest of the city", EQUAL);
	Hypothesis f("The prices of the cars are different from the rest of the city", DIFFERENT);

	/* TODO: Select a hypothesis and a null hypothesis from the above choices a-f and
	 * call test.setHypothesis(...) and test.setNullHypothesis(...) with the chosen
	 * variable.
	 */
	 test.setHypothesis(a);
	 test.setNullHypothesis(d);

	oneSampleTTest(arrayToVector(prices), distributionMean, confidenceLevel, test);
}

/**
 * \brief Executes a two sample T test for the "Planning" example on the exercise sheet.
 */
void StatisticalTesting::planning() {
	const double confidenceLevel = 0.95;

	const double myPlanner[] = {
		  90, 104, 142, 143, 121,
		 190,  92,  93, 166, 110,
		 191, 122, 129, 176, 110,
		  45,  78, 166, 173, 115,
		 197,  63, 156, 124,  98
	};
	const double baselinePlanner[] = {
		 56,  92, 145, 117, 121,
		 91, 147, 174, 122, 111,
		143, 142, 189, 129,  92,
		112, 122, 120, 125, 200,
		137, 147, 89, 101, 108
	};

	TestCase test;
	Hypothesis a("My planner produces longer paths than the baseline.", GREATER);
	Hypothesis b("My planner produces shorter paths than the baseline.", LESS);
	Hypothesis c("My plans are at most as long as the baseline plans.", AT_MOST);
	Hypothesis d("My plans are at least as long as the baseline plans.", AT_LEAST);
	Hypothesis e("My plans are as long as the baseline paths.", EQUAL);
	Hypothesis f("My plans have different lengths than the baseline paths.", DIFFERENT);

	test.setHypothesis(f);
	test.setNullHypothesis(e);
	twoSampleTTest(arrayToVector(myPlanner), arrayToVector(baselinePlanner),
			confidenceLevel, test);
}

/**
 * \brief Calculates the pooled variance of two vectors of numbers.
 * \param[in] values1 The first vector of numbers.
 * \param[in] values2 The second vector of numbers
 * \return The pooled sample variance.
 */
double StatisticalTesting::pooledVariance(const std::vector<double>& values1, const std::vector<double>& values2) {
	double pv = 0.0;
	// TODO
	double s1 = standardDeviation(values1);
	double s2 = standardDeviation(values2);
	int n1 = values1.size();
	int n2 = values2.size();
/*
	 int Mean =mean(values1);
	std::vector<double>::const_iterator it;
		for (it=values1.begin();it!=values1.end();it++) 	{
			s1 += pow(Mean-(*it),2);
		}
		s1 /= n1;

	Mean =mean(values2);
		for (it=values2.begin();it!=values2.end();it++) 	{
			s2 += pow(Mean-(*it),2);
		}
		s2 /= n2;

*/

	//pv =(double) (((n1-1) * s1*s1 )+((n2-1) * s2*s2 ))/(n1+n2-2);
	pv = (double)(n1*s1*s1+n2*s2*s2)/(n1+n2-2);
	return pv;
}

/**
 * \brief Calculates the standard error of two vectors of numbers with respect to each other.
 * \param[in] values1 The first vector of numbers.
 * \param[in] values2 The second vector of numbers
 * \return The standard error.
 */
double StatisticalTesting::standardError(const std::vector<double>& values1, const std::vector<double>& values2) {
	double SE = 0.0;
	// TODO
	double pv = pooledVariance(values1,values2);
	double n1 = values1.size();
	double n2 = values2.size();
	SE = sqrt(pv*(1/n1)*(1/n2));
	return SE;
}



/**
 * \brief Calculates the t value of two sample vectors.
 * \param[in] values1 The first vector of numbers.
 * \param[in] values2 The second vector of numbers.
 * \return The t value.
 */
double StatisticalTesting::tValueTwoSamples(const std::vector<double>& values1, const std::vector<double>& values2) {
	double t = 0.0;
	//TODO Calculate the t value with pooled variance for the two sample vectors.
	return t;
}





/**
 * \brief Runs a two-sample t test on two vectors of numbers.
 * \param[in] values1 The first vector of numbers.
 * \param[in] values2 The second vector of numbers.
 * \param[in] The confidence level (commonly used values: 0.95, 0.999)
 * \param[in] testCase The test case containing the hypothesis and null hypothesis chosen in germanStudentsTest()
 */
void StatisticalTesting::twoSampleTTest(const std::vector<double>& values1,
		const std::vector<double>& values2, const double& confidenceLevel, const TestCase& testCase) {
	/**
	 * TODO: Execute the two sample T test for the given vectors of numbers and either
	 * reject the null hypothesis or state that you cannot reject the null hypothesis.
	 */

	/* Available methods: Same as above for the one sample case. */

}



/* ======================================================================================= *
 * Lookup table methods, nothing to do below this line.                                    *
 * ======================================================================================= */

/**
 * \brief Implementation of the Z table that returns the cumulative distribution function
 * of the standard normal distribution.
 * \param[in] Z the Z score
 * \return The cumulative distribution function at Z.
 */
double StatisticalTesting::lookupZTable(const double& Z) {
	return boost::math::cdf(boost::math::normal(), Z);
}

/**
 * \brief Implementation of the T table that returns the quantile function of the
 * student's t distribution.
 * \param[in] degreeOfFreedom The degree of freedom of the distribution.
 * \param[in] confidenceLevel The confidence level.
 * \param[in] testType The test type, either ONE_SIDED or TWO_SIDED.
 * \return The quantile function at the given confidence level.
 */
double StatisticalTesting::lookupTTable(const size_t& degreeOfFreedom, const double& confidenceLevel, const TestType& testType) {
	if (testType == StatisticalTesting::ONE_SIDED) {
		return boost::math::quantile(boost::math::students_t(degreeOfFreedom), confidenceLevel);
	} else {
		if (confidenceLevel >= 0.5) {
			const double alpha = 1.0 - confidenceLevel;
			return boost::math::quantile(boost::math::students_t(degreeOfFreedom), 1.0 - alpha / 2);
		} else {
			const double alpha = confidenceLevel;
			return boost::math::quantile(boost::math::students_t(degreeOfFreedom), alpha / 2);
		}
	}
}

}  // namespace statistical_testing

