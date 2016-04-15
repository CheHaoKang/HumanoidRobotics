#include <gtest/gtest.h>
#include <particle_filter/ParticleFilter.h>
#include <cmath>
#include <boost/math/distributions/chi_squared.hpp>
#include <boost/math/distributions/students_t.hpp>

using namespace particle_filter;

TEST(ParticleFilter, gaussianProbability) {
	if (ParticleFilter::gaussianProbability(0, 1) == 0.0 && ParticleFilter::gaussianProbability(-2, 3) == 0.0) {
		FAIL() << "The method does not do anything.";
	}
	ASSERT_NEAR(0.398942, ParticleFilter::gaussianProbability(0, 1), 1e-5);
	ASSERT_NEAR(0.106483, ParticleFilter::gaussianProbability(-2, 3), 1e-5);
}

TEST(ParticleFilter, sampleFromGaussian) {
	const double alpha = 0.01;  // confidence level
	const double real_mean = 3.0;
	const double real_stdev = 2.0;
	const size_t n = 100;
	double samples[n];

	size_t pass = 0, mean_fail = 0, stdev_fail = 0;

	for (size_t j = 0; j < 1000; ++j) {
		double mean = 0.0, stdev = 0.0;
		for (size_t i = 0; i < n; ++i) {
			samples[i] = ParticleFilter::sampleFromGaussian(real_mean, real_stdev);
			mean += samples[i];
		}
		mean /= n;
		for (size_t i = 0; i < n; ++i) {
			stdev += (samples[i] - mean) * (samples[i] - mean);
		}
		stdev /= n;
		stdev = sqrt(stdev);

		// simple tests
		if (stdev < 1e-5) {
			if (mean == 0.0) {
				FAIL() << "The method does not do anything.";
			} else if (mean == samples[0]) {
				FAIL() << "The method always returns the mean.";
			} else {
				FAIL() << "The method always returns the same value " << samples[0] << ".";
			}
		}

		// Test mean
		boost::math::students_t_distribution<> student(n - 1);
		const double width = boost::math::quantile(boost::math::complement(student, 0.5 * alpha)) * real_stdev / sqrt((double) n);
		if(fabs(real_mean - mean) > width) {
			++mean_fail;
		} else {

			// Test standard deviation
			boost::math::chi_squared_distribution<> chi2(n - 1);
			const double lower_limit = boost::math::quantile(chi2, alpha / 2);
			const double upper_limit = boost::math::quantile(boost::math::complement(chi2, 0.5 * alpha));
			const double test_statistic = (n - 1) * (stdev / real_stdev) * (stdev / real_stdev);

			if (test_statistic < lower_limit || test_statistic > upper_limit) {
				++stdev_fail;
			} else {
				++pass;
			}
		}
	}

	if (((double) mean_fail) / ((double) pass + mean_fail + stdev_fail) > 0.05) {
		FAIL() << "The mean of the samples is outside the 99% confidence interval of the true mean in more than 5% of all cases.";
	}
	if (((double) stdev_fail) / ((double) pass + mean_fail + stdev_fail) > 0.05) {
		FAIL() << "The standard deviation of the samples is outside the 99% confidence interval of the true standard deviation in more than 5% of all cases.";
	}
}

TEST(ParticleFilter, initParticles) {
	std::vector<ParticleFilter::Particle> particles(100);
	ParticleFilter::initParticles(particles);
	ASSERT_EQ(100, particles.size());
	bool allXZero = true, allWeightsZero = true;
	for (std::vector<ParticleFilter::Particle>::const_iterator it = particles.begin(); it != particles.end(); ++it) {
		allXZero &= (it->x == 0.0);
		allWeightsZero &= (it->weight == 0.0);
		if (it->x < 0.0 || it->x > 10.0) {
			FAIL() << "There is a particle at position " << it-> x << ", which is outside the interval [0, 10].";
		}
	}
	if (allXZero && allWeightsZero) {
		FAIL() << "The method does not do anything.";
	}
	if (allXZero) {
		FAIL() << "The method does not set the position (x) of the particles.";
	}
	if (allWeightsZero) {
		FAIL() << "The method does not set the weights of the particles.";
	}
}

TEST(ParticleFilter, normalizeWeights) {
	std::vector<ParticleFilter::Particle> particles(20);
	for (size_t i = 0; i < particles.size(); ++i) {
		particles[i].weight = i + 1;
	}
	ParticleFilter::normalizeWeights(particles);
	double sum = 0.0;
	bool hasChanged = false;
	for (size_t i = 0; i < particles.size(); ++i) {
		hasChanged |= (particles[i].weight != i + 1);
		sum += particles[i].weight;
	}
	if (!hasChanged) {
		FAIL() << "The method does not change the weights at all.";
	}
	if (fabs(sum - 1.0) > 1e-5) {
		FAIL() << "The particle weights do not sum up to 1.";
	}
	double s = 2.0 / ((double) (particles.size() * (particles.size() + 1)));
	for (size_t i = 0; i < particles.size(); ++i) {
		ASSERT_NEAR((i + 1.0) * s, particles[i].weight, 1e-5);
	}
}

TEST(ParticleFilter, integrateMotion) {
	const double alpha = 0.01; // 99% confidence
	const double ux = 2.0;
	const double real_stdev = 1.0;
	const double mean_before = 5.0;
	const double spread = 2.0;
	size_t n = 100;
	size_t pass = 0, mean_fail = 0, stdev_fail = 0;

	std::vector<ParticleFilter::Particle> particles(n);
	const double sx = mean_before - spread;
	const double dx = 2.0 * spread / (particles.size() - 1);

	for (size_t j = 0; j < 1000; ++j) {
		for (size_t i = 0; i < particles.size(); ++i) {
			particles[i].x = sx + dx * i;
		}
		ParticleFilter::integrateMotion(particles, ux, real_stdev);
		ASSERT_EQ(100, particles.size());
		double mean = 0.0, stdev = 0.0;
		bool hasChanged = false;
		for (size_t i = 0; i < particles.size(); ++i) {
			const double valueBefore = sx + dx * i;
			hasChanged |= (particles[i].x != valueBefore);
			mean += particles[i].x - valueBefore;
		}
		mean /= (double) n;
		if (!hasChanged) {
			FAIL() << "The method does not do anything.";
		}

		for (size_t i = 0; i < particles.size(); ++i) {
			const double valueBefore = sx + dx * i;
			stdev += (particles[i].x - valueBefore - mean) * (particles[i].x - valueBefore - mean);
		}
		stdev = sqrt(stdev / ((double) n));

		// Simple tests
		if (stdev < 1e-5) {
			if (mean == ux) {
				FAIL() << "The method only shifts the particles according to the odometry, but does not sample from the motion model.";
			} else {
				FAIL() << "The method shifts all particles by " << (mean - mean_before) << " instead of sampling from the motion model.";
			}
		}

		// Test mean
		boost::math::students_t_distribution<> student(n - 1);
		const double width = boost::math::quantile(boost::math::complement(student, 0.5 * alpha)) * real_stdev / sqrt((double) n);
		if(fabs(ux - mean) > width) {
			++mean_fail;
		} else {
			// Test standard deviation
			boost::math::chi_squared_distribution<> chi2(n - 1);
			const double lower_limit = boost::math::quantile(chi2, alpha / 2);
			const double upper_limit = boost::math::quantile(boost::math::complement(chi2, 0.5 * alpha));
			const double test_statistic = (n - 1) * (stdev / real_stdev) * (stdev / real_stdev);

			if (test_statistic < lower_limit || test_statistic > upper_limit) {
				++stdev_fail;
			} else {
				++pass;
			}
		}
	}

	if (((double) mean_fail) / ((double) pass + mean_fail + stdev_fail) > 0.05) {
		FAIL() << "The mean of the movement is outside of the 99% confidence interval of the real movement in more than 5% of all cases.";
	}
	if (((double) stdev_fail) / ((double) pass + mean_fail + stdev_fail) > 0.05) {
		FAIL() << "The standard deviation of the samples is outside the 99% confidence interval of the true standard deviation in more than 5% of all cases.";
	}
}

TEST(ParticleFilter, getDistanceToNearestLight) {
	double values[] = {2, 1.5, 1, 0.5, 0, 0.5, 1, 1.5, 2, 1.5, 1, 0.5, 0, 0.5, 1, 0.5, 0, 0.5, 1, 1.5, 2};
	double x = 0.0, dx = 0.5;
	bool hasNonzero = false;
	for (size_t i = 0; i < sizeof(values) / sizeof(values[0]) && !hasNonzero; ++i) {
		hasNonzero |= ParticleFilter::getDistanceToNearestLight(x) != 0.0;
		x += dx;
	}
	if (!hasNonzero) {
		FAIL() << "The method does not do anything.";
	}
	x = 0.0;
	for (size_t i = 0; i < sizeof(values) / sizeof(values[0]); ++i) {
		ASSERT_NEAR(values[i], ParticleFilter::getDistanceToNearestLight(x), 1e-5);
		x += dx;
	}
}

TEST(ParticleFilter, integrateObservation) {
	std::vector<ParticleFilter::Particle> particles(4);
	for (size_t i = 0; i < particles.size(); ++i) {
		particles[i].weight = 1.0 / particles.size();
	}
	particles[0].x = 2.0;
	particles[1].x = 3.0;
	particles[2].x = 4.0;
	particles[3].x = 5.0;

	ParticleFilter::integrateObservation(particles, 1.0, 1.0);
	bool hasChanged = false;
	double sum = 0.0;
	for (size_t i = 0; i < particles.size(); ++i) {
		hasChanged |= (particles[i].weight != 1.0 / particles.size());
		sum += particles[i].weight;
	}
	if (!hasChanged) {
		FAIL() << "The method does not update the particle weights.";
	}
	if (fabs(sum - 1.0) > 1e-4) {
		FAIL() << "The sum of the weights is " << sum << ", but it should be normalized to 1.";
	}

	ASSERT_NEAR(0.18877, particles[0].weight, 1e-4);
	ASSERT_NEAR(0.31123, particles[1].weight, 1e-4);
	ASSERT_NEAR(0.18877, particles[2].weight, 1e-4);
	ASSERT_NEAR(0.31123, particles[3].weight, 1e-4);
}

TEST(ParticleFilter, resample) {
	std::vector<ParticleFilter::Particle> particles(4);
	particles[0].x = 1.0;
	particles[1].x = 2.0;
	particles[2].x = 3.0;
	particles[3].x = 4.0;
	particles[0].weight = 0.0;
	particles[1].weight = 0.25;
	particles[2].weight = 0.75;
	particles[3].weight = 0.0;
	const std::vector<ParticleFilter::Particle> newParticles = ParticleFilter::resample(particles);
	if (newParticles.empty()) {
		FAIL() << "The method returns an empty vector.";
	}
	ASSERT_EQ(particles.size(), newParticles.size());

	size_t count[4] = {0, 0, 0, 0};
	bool hasInvalid = false;
	for (size_t i = 0; i < newParticles.size(); ++i) {
		if (newParticles[i].x == 1.0) {
			++count[0];
		} else if (newParticles[i].x == 2.0) {
			++count[1];
		} else if (newParticles[i].x == 3.0) {
			++count[2];
		} else if (newParticles[i].x == 4.0) {
			++count[3];
		} else {
			hasInvalid = true;
		}
	}
	if (hasInvalid) {
		FAIL() << "The method returned a particle that did not exist in the original particle set.";
	}
	if (count[0] == 1 && count[1] == 1 && count[2] == 1 && count[3] == 1) {
		FAIL() << "The method returned the original particle set without resampling.";
	}
	if (count[0] > 0 || count[3] > 0) {
		FAIL() << "The method returned a particle whose original weight was 0. Such particles should not be in the resampled set.";
	}
	ASSERT_EQ(1, count[1]);
	ASSERT_EQ(3, count[2]);
}

int main(int argc, char *argv[]) {
	::testing::InitGoogleTest(&argc, argv);
	srand(time(0));
	return RUN_ALL_TESTS();
}
