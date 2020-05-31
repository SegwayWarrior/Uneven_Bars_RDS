# select parameter
## Instruction:

1. The bar location is (2.9,1.2). It is derived by the real gymnaistic uneven bar location.
2. real_success_parameter.m uses a big loop to test all the possible combination of six value: theta_1, theta_2, theta_3, dtheta_1, dtheta_2, dtheta_3. The range and interval are not big for the calculating demonsion. The param which could success appoarch another bar range is stored in parameter.mat.
3. success_parameter is used to select the success parameters. The parameter which are success to grab another bar is store in success_para.mat. Then the function draw a picture about all the trails which could grab the bar (2.9,1.2);
4. In my example, I test about 57000 combinations of value, and about 1800 are success to grab the bar.
5. The example picture is all the trials which could hit another bar location.
![](example1.jpg)