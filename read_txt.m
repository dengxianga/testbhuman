%% Read file into matrix
filename = 'walklog.txt';
delimiter = ',';

%%
fid = fopen('walklog.txt');
labels = fgetl(fid);
label_cell = strsplit(labels, delimiter);
label_cell(~cellfun('isempty', label_cell)); 
buffer = fread(fid, Inf); 
fclose(fid);

fid = fopen('temp.txt', 'w');  
fwrite(fid, buffer);                        
fclose(fid);

label_cell = label_cell(2:end);
%%
data = dlmread('temp.txt', delimiter);

figure;
for i = 1:22
    subplot(4, 6, i);
    hold on
    plot(data(:, i));
    hold off
    title(label_cell(i));
    xlabel('Time [s]');
end
