# Exercise 18: Statistical testing

Statistical testing is an important tool for assessing the quality of algorithms in robotics. In this
exercise, we will use Z tests and T tests to make confident statements about samples.

## Z Test

In a nationwide test, German students reached on average 100 points with a standard deviation 
of 12. You are given the results of 55 students from Bonn. Someone claims that the Bonn students
performed significantly worse than the average German students. Use the Z test to support that
claim or to show that the data is insufficient for deciding:

1. Implement the methods for calculating the mean and the Z score.
2. Complete the method `germanStudentsTest` by choosing the correct hypothesis `H_1` 
   and null hypothesis `H_0`.
3. Implement the method `oneSampleZTest` for performing the Z test. For looking up the Z score
in the Z table, a method named `lookupZTable` is already provided in the code. It returns
the value of the cumulative distribution function for given Z, i.e., the area below the Gaussian
marked (see exercise sheet for a figure).
Make a distinction of cases for the three possible cases of left-tailed, right-tailed, and two-tailed
tests. In the end, either reject the null hypothesis or state that you cannot reject the null
hypothesis based on the data.

## One-sample T Test
A study found that the average price of a car in a particular city is $12,000. Five cars park in front
of a house with the prices listed in `cars()`. A car thief claims that the cars are more expensive
than in the rest of the city. Use the one-sample T test to support that claim or to show that the
data is insufficient for deciding:

4. Implement the methods for calculating the sample standard deviation and the t value. 
   Estimate the standard deviation from the sample.
5. Complete the method `cars` by choosing the correct hypothesis `H_1` and null hypothesis `H_0`.
6. Implement the one-sample T-Test. For looking up the values of the T table, the existing code
already provides the method `lookupTTable`. The T table provides the quantile function of
the Student's t distribution, i.e., the inverse of the cumulative distribution function. See the
figure on the exercise sheet for an illustration. Distinguish again between left-tailed, right-tailed, and two-tailed tests.

## Two-sample T Test
You implemented a path planning algorithm and would like to compare it to the standard approach
from the literature (the "baseline approach"). The method `planning()` lists the path lengths for 25
randomly generated scenarios both with your planner and the baseline planner. You claim that the
path lengths of your planner are different from the baseline planner's paths. Use the two-sample T
test to support your claim or to show that the data is insufficient for deciding as follows:
7. Complete the method planning by choosing the correct hypothesis `H_1` and null 
   hypothesis `H_0`.
8. The variance of the distribution has to be estimated from two samples simultaneously 
   instead of from one sample only. To do so, compute the pooled variance 
   in `pooledVariance` according to the formula on the exercise sheet.
9. Compute the standard error of the two samples with respect to each other according to 
   the formula on the exercise sheet.
10. Implement the method `tValueTwoSamples` that computes the t value according to the
    formula on the exercise sheet.
11. Implement the method `tValueTwoSamples` using the t value from the previous method 
    and `N_1 + N_2 - 2` degrees of freedom. The rest of the code will be identical to 
    the one-sample test.
