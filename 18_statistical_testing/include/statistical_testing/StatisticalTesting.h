#ifndef STATISTICAL_TESTING_H_
#define STATISTICAL_TESTING_H_

#include <vector>
#include <boost/math/distributions/normal.hpp>
#include <boost/math/distributions/students_t.hpp>

namespace statistical_testing {

template<typename T, size_t N>
std::vector<T> arrayToVector(const T (&array)[N]) {
	return std::vector<T>(array, array + N);
}

enum TestDirection {
	EQUAL, DIFFERENT, LESS, GREATER, AT_LEAST, AT_MOST
};

class Hypothesis {
public:
	Hypothesis(const std::string& hypothesis, const TestDirection& direction)
        : hypothesis(hypothesis), otherHypothesis(NULL), direction(direction),
		  isNullHypothesis(false) {}
	virtual ~Hypothesis() {}
	virtual void reject() const {
		std::cout << "Rejecting the "
				  << (isNullHypothesis ? "null hypothesis" : "hypothesis")
				  << " \"" << hypothesis << "\", "
				  << std::endl;
		std::cout << "hence, the "
				  << (isNullHypothesis ? "hypothesis" : "other hypothesis")
				  << " \"" << otherHypothesis->hypothesis << "\" is probably true."
				  << std::endl;
	}
	virtual void cannotReject() const {
		std::cout << "Cannot reject the "
				<< (isNullHypothesis ? "null hypothesis" : "hypothesis")
				<< " \"" << hypothesis << "\", "
				<< std::endl;
		std::cout << "hence, we cannot decide whether the "
				<< (isNullHypothesis ? "hypothesis" : "other hypothesis")
			    << " \"" << otherHypothesis->hypothesis << "\" is true or false."
				<< std::endl;
	}

	const TestDirection getDirection() const {
		return direction;
	}
	const bool isOneSided() const {
		return direction != EQUAL && direction != DIFFERENT;
	}

private:
	const std::string hypothesis;
	const Hypothesis * otherHypothesis;
	const TestDirection direction;
	bool isNullHypothesis;
	friend class TestCase;

};

class TestCase {
public:
	TestCase() : hypothesis(NULL), nullHypothesis(NULL) {}
	const Hypothesis& getHypothesis() const {
		if (!hypothesis) {
			throw std::runtime_error("The hypothesis has not been set yet. Call setHypothesis(...) first.");
		}
		return *hypothesis;
	}

	const Hypothesis& getNullHypothesis() const {
		if (!nullHypothesis) {
			throw std::runtime_error("The null hypothesis has not been set yet. Call setNullHypothesis(...) first.");
		}

		return *nullHypothesis;
	}

	void setHypothesis(Hypothesis& hypothesis) {
	    if (hypothesis.getDirection() == EQUAL || hypothesis.getDirection() == AT_LEAST || hypothesis.getDirection() == AT_MOST) {
	        throw std::invalid_argument("It is mathematically impossible to show this hypothesis. Rule of thumb: The hypothesis must not contain an equal sign.");
	    }
		this->hypothesis = &hypothesis;
		connect();
	}

	void setNullHypothesis(Hypothesis& nullHypothesis) {
	    if (nullHypothesis.getDirection() == LESS || nullHypothesis.getDirection() == GREATER || nullHypothesis.getDirection() == DIFFERENT) {
	        throw std::invalid_argument("It is mathematically impossible to reject this null hypothesis. Rule of thumb: The null hypothesis must contain an equal sign.");
	    }
		this->nullHypothesis = &nullHypothesis;
		connect();
	}

	void connect() {
		if (nullHypothesis && hypothesis) {
			bool match;
			switch (hypothesis->getDirection()) {
			case LESS:      match = nullHypothesis->getDirection() == AT_LEAST;  break;
			case GREATER:   match = nullHypothesis->getDirection() == AT_MOST;   break;
			case DIFFERENT: match = nullHypothesis->getDirection() == EQUAL;     break;
			}
			if (!match) {
				throw std::invalid_argument("The hypothesis and the null hypothesis have to be complements");
			}
			hypothesis->otherHypothesis = nullHypothesis;
			nullHypothesis->otherHypothesis = hypothesis;
		}
	}

private:
	Hypothesis * hypothesis;
	Hypothesis * nullHypothesis;
};


class StatisticalTesting
{
public:
	StatisticalTesting() {};
   	virtual ~StatisticalTesting() {};

   	enum TestType { ONE_SIDED, TWO_SIDED };

   	virtual double mean(const std::vector<double>& values);
   	virtual double standardDeviation(const std::vector<double>& values);
   	virtual double zScore(const std::vector<double>& values, const double& distributionMean, const double& distributionStandardDeviation);

   	virtual void oneSampleZTest(const std::vector<double>& values, const double& distributionMean, const double& distributionStandardDeviation, const double& confidenceLevel, const TestCase& testCase);
   	virtual void germanStudentsTest();

   	double tValue(const std::vector<double>& values, const double& distributionMean);
   	virtual void oneSampleTTest(const std::vector<double>& values, const double& distributionMean, const double& confidenceLevel, const TestCase& testCase);
   	virtual void cars();

   	virtual double pooledVariance(const std::vector<double>& values1, const std::vector<double>& values2);
   	virtual double standardError(const std::vector<double>& values1, const std::vector<double>& values2);
   	virtual double tValueTwoSamples(const std::vector<double>& values1, const std::vector<double>& values2);
   	virtual void twoSampleTTest(const std::vector<double>& values1, const std::vector<double>& values2,
   			const double& confidenceLevel, const TestCase& testCase);

   	void planning();

   	virtual double lookupZTable(const double& Z);
   	virtual double lookupTTable(const size_t& degreeOfFreedom, const double& confidenceLevel, const TestType& testType);

};

}  // namespace statistical_testing

#endif  // STATISTICAL_TESTING_H_
