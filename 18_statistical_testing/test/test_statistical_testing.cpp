#include <gtest/gtest.h>
#include <statistical_testing/StatisticalTesting.h>

#define TEST_PROVIDED_FUNCTIONS 0

using namespace statistical_testing;

TEST(StatisticalTesting, mean) {
	const double tolerance = 1e-5;
	const double values[] = {0.5, 0.4, 0.6, 0.4};
	const double mean = StatisticalTesting().mean(arrayToVector(values));
	if (mean == 0.0)
		FAIL() << "The method does not return the mean.";
	ASSERT_NEAR(0.475, mean, tolerance);
}


TEST(StatisticalTesting, zScore) {
	const double tolerance = 1e-5;
	const double values[] = {0.5, 0.4, 0.6, 0.4};
	const double z = StatisticalTesting().zScore(arrayToVector(values), 0.5, 0.12);
	if (z == 0.0)
		FAIL() << "The method does not return the Z score.";
	if (fabs(z - 0.4166667) < tolerance) {
		FAIL() << "The sign is incorrect.";
	}
	ASSERT_NEAR(-0.4166667, z, tolerance);
}

class TestHypothesis : public Hypothesis {
public:
	TestHypothesis(const TestDirection& direction) : Hypothesis(std::string(), direction), rejected(false), notRejected(false) {}
	virtual void reject() const {
		if (rejected || notRejected) {
			FAIL() << "reject() and cannotReject() must only be called once per hypothesis";
		} else {
			rejected = true;
		}
	}
	virtual void cannotReject() const {
		if (rejected || notRejected) {
			FAIL() << "reject() and cannotReject() must only be called once per hypothesis";
		} else {
			notRejected = true;
		}
	}

	bool wasNotRejected() const {
		return notRejected;
	}

	bool wasRejected() const {
		return rejected;
	}

private:
	mutable bool rejected;
	mutable bool notRejected;
};

class DOFIntercept : public StatisticalTesting {
public:
	DOFIntercept(const size_t degreeOfFreedom) : dof(degreeOfFreedom) {}
	double lookupTTable(const size_t& degreeOfFreedom, const double& confidenceLevel, const TestType& testType) {
		if (degreeOfFreedom != dof) {
			ADD_FAILURE() << "The degrees of freedom are incorrect when looking up the value in the t table.";
		}
		return StatisticalTesting::lookupTTable(degreeOfFreedom, confidenceLevel, testType);
	}

private:
	size_t dof;
};

class HelperError : public std::exception {
public:
	const char * const reason;
	HelperError(const char * const reason) : std::exception(), reason(reason) {};
	const char * what() const throw() { return reason; }
};

bool testHelper(const TestDirection& hDir, const TestDirection& nDir, const double& distMean, bool tTest) {
	const double confidence = 0.95;
	const double distStdev = 0.12;
	const double values[] = {0.5, 0.4, 0.6, 0.4};
	DOFIntercept st(3);
	TestCase tc;
	TestHypothesis h(hDir);
	TestHypothesis n(nDir);
	tc.setHypothesis(h);
	tc.setNullHypothesis(n);
	if (tTest) {
		st.oneSampleTTest(arrayToVector(values), distMean, confidence, tc);
	} else {
		st.oneSampleZTest(arrayToVector(values), distMean, distStdev, confidence, tc);
	}
	if (h.wasRejected() || h.wasNotRejected()) {
		throw HelperError("The method called reject() / cannotReject() on the hypothesis, but these methods are only relevant for the null hypothesis.");
	}
	if (!n.wasRejected() && !n.wasNotRejected()) {
		throw HelperError("The method must call reject() or cannotReject() on the null hypothesis.");
	}
	return n.wasRejected();
}

bool testHelper(const TestDirection& hDir, const TestDirection& nDir, const std::vector<double>& a, const std::vector<double>& b) {
	const double confidence = 0.95;
	DOFIntercept st(a.size() + b.size() - 2);
	TestCase tc;
	TestHypothesis h(hDir);
	TestHypothesis n(nDir);
	tc.setHypothesis(h);
	tc.setNullHypothesis(n);
	st.twoSampleTTest(a, b, confidence, tc);
	if (h.wasRejected() || h.wasNotRejected()) {
		throw HelperError("The method called reject() / cannotReject() on the hypothesis, but these methods are only relevant for the null hypothesis.");
	}
	if (!n.wasRejected() && !n.wasNotRejected()) {
		throw HelperError("The method must call reject() or cannotReject() on the null hypothesis.");
	}
	return n.wasRejected();
}

class TestCaseIntercept : public StatisticalTesting {
private:
	TestCase testCase;
	bool testCalled;

public:
	TestCaseIntercept() : testCalled(false) {}
	void oneSampleZTest(const std::vector<double>& values,
			const double& distributionMean, const double& distributionStandardDeviation,
			const double& confidenceLevel, const TestCase& tc) {
		testCase = tc;
		testCalled = true;
	}
	void oneSampleTTest(const std::vector<double>& values, const double& distributionMean,
			const double& confidenceLevel, const TestCase& tc) {
		testCase = tc;
		testCalled = true;
	}
	void twoSampleTTest(const std::vector<double>& values1, const std::vector<double>& values2,
	   			const double& confidenceLevel, const TestCase& tc) {
		testCase = tc;
		testCalled = true;
	}

	const TestCase& getTestCase() {
		return testCase;
	}
	bool wasTestCalled() {
		return testCalled;
	}

};

TEST(StatisticalTesting, germanStudentsTest) {
	TestCaseIntercept tci;
	try {
		tci.germanStudentsTest();
	} catch (const std::invalid_argument& e) {
		FAIL() << e.what();
	}
	if (!tci.wasTestCalled()) {
		FAIL() << "The method did not call oneSampleZTest() as it was originally in the provided code.";
	}
	const TestCase& testCase = tci.getTestCase();
	try {
		const Hypothesis& hypothesis = testCase.getHypothesis();
		if (hypothesis.getDirection() != LESS) {
			FAIL() << "The method chose the wrong hypothesis.";
		}
	} catch(const std::runtime_error& e) {
		FAIL() << "The method does not set the hypothesis.";
	}
	try {
		const Hypothesis& nullHypothesis = testCase.getNullHypothesis();
		if (nullHypothesis.getDirection() != AT_LEAST) {
			FAIL() << "The method chose the wrong hypothesis.";
		}
	} catch(const std::runtime_error& e) {
		FAIL() << "The method does not set the null hypothesis.";
	}
}

TEST(StatisticalTesting, oneSampleTZest) {
	try {
		if (!testHelper(DIFFERENT, EQUAL, 10.0, false))
			FAIL() << "The method did not reject the null hypothesis 'the distributions are equal' although the sample mean is significantly lower than the confidence interval of the distribution mean.";

		if (!testHelper(DIFFERENT, EQUAL, -10.0, false))
			FAIL() << "The method did not reject the null hypothesis 'the distributions are equal' although the sample mean is significantly higher than the confidence interval of the distribution mean.";

		if (testHelper(DIFFERENT, EQUAL, 0.55, false))
			FAIL() << "The method rejected the null hypothesis 'the distributions are equal' although the sample mean inside the confidence interval of the distribution mean.";

		if (!testHelper(LESS, AT_LEAST, 10.0, false))
			FAIL() << "The method did not reject the null hypothesis 'at least' although the sample mean is significantly lower than the confidence interval.";

		if (testHelper(LESS, AT_LEAST, 0.55, false))
			FAIL() << "The method rejected the null hypothesis 'at least' although the sample mean is inside the confidence interval of the distribution mean.";

		if (!testHelper(GREATER, AT_MOST, -10.0, false))
			FAIL() << "The method did not reject the null hypothesis 'at most' although the sample mean is significantly higher than the confidence interval.";

		if (testHelper(GREATER, AT_MOST, 0.55, false))
			FAIL() << "The method rejected the null hypothesis 'at most' although the sample mean is inside the confidence interval of the distribution mean.";
	} catch (const HelperError& e) {
		FAIL() << e.what();
	}
}


TEST(StatisticalTesting, standardDeviation) {
	const double tolerance = 1e-5;
	const double values[] = {0.5, 0.4, 0.6, 0.4};
	const double sigma = StatisticalTesting().standardDeviation(arrayToVector(values));
	if (sigma == 0.0)
		FAIL() << "The method does not return the standard deviation.";
	if (fabs(sigma - 0.0829156) < tolerance) {
		FAIL() << "The method calculates the population standard deviation instead of the sample standard deviation.";
	}
	ASSERT_NEAR(0.0957427, sigma, tolerance);
}


TEST(StatisticalTesting, tValue) {
	const double tolerance = 1e-5;
	const double values[] = {0.5, 0.4, 0.6, 0.4};
	const double t = StatisticalTesting().tValue(arrayToVector(values), 0.5);
	if (t == 0.0)
		FAIL() << "The method does not return the t value.";
	if (fabs(t - 0.522232) < tolerance) {
		FAIL() << "The sign is incorrect.";
	}
	ASSERT_NEAR(-0.522232, t, tolerance);
}


TEST(StatisticalTesting, oneSampleTTest) {
	try {
		if (!testHelper(DIFFERENT, EQUAL, 10.0, true))
			FAIL() << "The method did not reject the null hypothesis 'the distributions are equal' although the sample mean is significantly lower than the confidence interval of the distribution mean.";

		if (!testHelper(DIFFERENT, EQUAL, -10.0, true))
			FAIL() << "The method did not reject the null hypothesis 'the distributions are equal' although the sample mean is significantly higher than the confidence interval of the distribution mean.";

		if (testHelper(DIFFERENT, EQUAL, 0.55, true))
			FAIL() << "The method rejected the null hypothesis 'the distributions are equal' although the sample mean inside the confidence interval of the distribution mean.";

		if (!testHelper(LESS, AT_LEAST, 10.0, true))
			FAIL() << "The method did not reject the null hypothesis 'at least' although the sample mean is significantly lower than the confidence interval.";

		if (testHelper(LESS, AT_LEAST, 0.55, true))
			FAIL() << "The method rejected the null hypothesis 'at least' although the sample mean is inside the confidence interval of the distribution mean.";

		if (!testHelper(GREATER, AT_MOST, -10.0, true))
			FAIL() << "The method did not reject the null hypothesis 'at most' although the sample mean is significantly higher than the confidence interval.";

		if (testHelper(GREATER, AT_MOST, 0.55, true))
			FAIL() << "The method rejected the null hypothesis 'at most' although the sample mean is inside the confidence interval of the distribution mean.";
	} catch (const HelperError& e) {
		FAIL() << e.what();
	}
}

TEST(StatisticalTesting, cars) {
	TestCaseIntercept tci;
	try {
		tci.cars();
	} catch (const std::invalid_argument& e) {
		FAIL() << e.what();
	}
	if (!tci.wasTestCalled()) {
		FAIL() << "The method did not call oneSampleTTest() as it was originally in the provided code.";
	}
	const TestCase& testCase = tci.getTestCase();
	try {
		const Hypothesis& hypothesis = testCase.getHypothesis();
		if (hypothesis.getDirection() != GREATER) {
			FAIL() << "The method chose the wrong hypothesis.";
		}
	} catch(const std::runtime_error& e) {
		FAIL() << "The method does not set the hypothesis.";
	}
	try {
		const Hypothesis& nullHypothesis = testCase.getNullHypothesis();
		if (nullHypothesis.getDirection() != AT_MOST) {
			FAIL() << "The method chose the wrong hypothesis.";
		}
	} catch(const std::runtime_error& e) {
		FAIL() << "The method does not set the null hypothesis.";
	}
}

TEST(StatisticalTesting, planning) {
	TestCaseIntercept tci;
	try {
		tci.planning();
	} catch (const std::invalid_argument& e) {
		FAIL() << e.what();
	}
	if (!tci.wasTestCalled()) {
		FAIL() << "The method did not call oneSampleTTest() as it was originally in the provided code.";
	}
	const TestCase& testCase = tci.getTestCase();
	try {
		const Hypothesis& hypothesis = testCase.getHypothesis();
		if (hypothesis.getDirection() != DIFFERENT) {
			FAIL() << "The method chose the wrong hypothesis.";
		}
	} catch(const std::runtime_error& e) {
		FAIL() << "The method does not set the hypothesis.";
	}
	try {
		const Hypothesis& nullHypothesis = testCase.getNullHypothesis();
		if (nullHypothesis.getDirection() != EQUAL) {
			FAIL() << "The method chose the wrong hypothesis.";
		}
	} catch(const std::runtime_error& e) {
		FAIL() << "The method does not set the null hypothesis.";
	}
}

TEST(StatisticalTesting, pooledVariance) {
	const double tolerance = 1e-5;
	StatisticalTesting st;
	const double a[] = {
		2.7959921330, 2.2428416490, 3.1676791880, 3.6807657190, 2.6304584700,
		2.1909001400, 3.0464827570, 2.7868094390, 3.1846238930, 2.6547259730
	};
	const double b[] = {
		4.1630082650, 2.5421340690, 2.4034877270, 4.6664957890, 4.5920467810,
		2.2875798620, 2.0546940080, 3.7281599550, 2.1011164590, 2.5153466510
	};
	const double pv = st.pooledVariance(arrayToVector(a), arrayToVector(b));
	if (pv == 0.0) {
		FAIL() << "The method does not return the pooled variance.";
	}
	ASSERT_NEAR(0.66275, pv, tolerance);
}

TEST(StatisticalTesting, standardError) {
	const double tolerance = 1e-5;
	StatisticalTesting st;
	const double a[] = {
		2.7959921330, 2.2428416490, 3.1676791880, 3.6807657190, 2.6304584700,
		2.1909001400, 3.0464827570, 2.7868094390, 3.1846238930, 2.6547259730
	};
	const double b[] = {
		4.1630082650, 2.5421340690, 2.4034877270, 4.6664957890, 4.5920467810,
		2.2875798620, 2.0546940080, 3.7281599550, 2.1011164590, 2.5153466510
	};
	const double SE = st.standardError(arrayToVector(a), arrayToVector(b));
	if (SE == 0.0) {
		FAIL() << "The method does not return the standard error.";
	}
	ASSERT_NEAR(0.364075, SE, tolerance);
}

TEST(StatisticalTesting, tValueTwoSamples) {
	const double tolerance = 1e-5;
	StatisticalTesting st;
	const double a[] = {
		2.7959921330, 2.2428416490, 3.1676791880, 3.6807657190, 2.6304584700,
		2.1909001400, 3.0464827570, 2.7868094390, 3.1846238930, 2.6547259730
	};
	const double b[] = {
		4.1630082650, 2.5421340690, 2.4034877270, 4.6664957890, 4.5920467810,
		2.2875798620, 2.0546940080, 3.7281599550, 2.1011164590, 2.5153466510
	};
	const double t = st.tValueTwoSamples(arrayToVector(a), arrayToVector(b));
	if (t == 0.0) {
		FAIL() << "The method does not return the t value.";
	}
	ASSERT_NEAR(-0.734131, t, tolerance);
}

TEST(StatisticalTesting, twoSampleTTest) {
	const double low[] = {
		 2.75180, 0.31053, 3.89560, 7.50680, 5.69120,
		 1.67890, 2.59300, 3.68820, 3.71870, 3.81500
	};
	const double mid[] = {
		 5.43140, 6.93250, 6.45530, 5.31760, 4.06310,
		 2.28130, 2.22990, 2.67140, 5.89910, 5.43760
	};
	const double high[] = {
		 7.83200, 9.65810, 8.08390, 8.53870, 7.37140,
		 4.28780, 9.36150, 7.75330, 9.97670, 8.83360
	};
	const std::vector<double>& vlow(arrayToVector(low));
	const std::vector<double>& vmid(arrayToVector(mid));
	const std::vector<double>& vhigh(arrayToVector(high));

	try {
		if (!testHelper(DIFFERENT, EQUAL, vlow, vhigh))
			FAIL() << "The method did not reject the null hypothesis 'the distributions are equal' although the first sample mean is significantly lower than the second sample mean.";

		if (!testHelper(DIFFERENT, EQUAL, vhigh, vlow))
			FAIL() << "The method did not reject the null hypothesis 'the distributions are equal' although the first sample mean is significantly higher than the second sample mean.";

		if (testHelper(DIFFERENT, EQUAL, vlow, vmid))
			FAIL() << "The method rejected the null hypothesis 'the distributions are equal' although the sample means are close by.";

		if (!testHelper(LESS, AT_LEAST, vlow, vhigh))
			FAIL() << "The method did not reject the null hypothesis 'at least' although the first sample mean is significantly lower than the second sample mean.";

		if (testHelper(LESS, AT_LEAST, vlow, vmid))
			FAIL() << "The method rejected the null hypothesis 'at least' although the sample means are close by.";

		if (!testHelper(GREATER, AT_MOST, vhigh, vlow))
			FAIL() << "The method did not reject the null hypothesis 'at most' although the first sample mean is significantly higher than the second sample mean.";

		if (testHelper(GREATER, AT_MOST, vlow, vmid))
			FAIL() << "The method rejected the null hypothesis 'at most' although the sample means are close by.";
	} catch (const HelperError& e) {
		FAIL() << e.what();
	}
}

#if TEST_PROVIDED_FUNCTIONS
TEST(StatisticalTesting, lookupZTable) {
	const double& tolerance = 1e-4;
	StatisticalTesting st;
	ASSERT_NEAR(0.99865, st.lookupZTable(3.0), tolerance);
	ASSERT_NEAR(0.5, st.lookupZTable(0.0), tolerance);
	ASSERT_NEAR(0.158655, st.lookupZTable(-1.0), tolerance);
	ASSERT_NEAR(0.9545, st.lookupZTable(2.0) - st.lookupZTable(-2.0), tolerance);
}

TEST(StatisticalTesting, lookupTTable) {
	const double& tolerance = 1e-4;
	StatisticalTesting st;
	ASSERT_NEAR(0.816497, st.lookupTTable(2, 0.75, StatisticalTesting::ONE_SIDED), tolerance);
	ASSERT_NEAR(1.943180, st.lookupTTable(6, 0.95, StatisticalTesting::ONE_SIDED), tolerance);
	ASSERT_NEAR(0.816497, st.lookupTTable(2, 0.50, StatisticalTesting::TWO_SIDED), tolerance);
	ASSERT_NEAR(1.943180, st.lookupTTable(6, 0.90, StatisticalTesting::TWO_SIDED), tolerance);

}
#endif


int main(int argc, char *argv[]) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
