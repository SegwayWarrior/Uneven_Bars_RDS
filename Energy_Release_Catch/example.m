sampleTime = 0.01;
endTime = 10;
numberOfSamples = endTime * 1/sampleTime +1;
timeVector = (0:numberOfSamples) * sampleTime;

signal_1 = timeseries(sin(timeVector)*10,timeVector);
signal_2 = timeseries(rand(size(timeVector)),timeVector);

busSignal.busElement_1 = timeseries(cos(timeVector)*2,timeVector);
busSignal.busElement_2 = timeseries(randn(size(timeVector)),timeVector);

busInfo = Simulink.Bus.createObject(busSignal);