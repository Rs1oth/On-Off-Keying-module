%MATLAB code to graph a heatmap

z = dlmread('BER2.dat');
x = 1:200;
y = 1:200;

colormap('hot');
imagesc(z);
colorbar;

title('Throughput values with interference on whole System');
xlabel('Interferers Power [0-200 dbm]');
ylabel('Power [0-200 dbm]');
zlabel('Throughput');


%in the script used to output data points the following code should be added:
%     std::ofstream myfile;
%     myfile.open("sim.txt");
%     // calculations and other code goes here
%     in first loop{
%     myfile << throughput << " ";
%      }
%     in second loop{
%     << std::endl;
%     }
%     myfile.close();
