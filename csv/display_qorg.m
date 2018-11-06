function display_qorg(filename)

% add matlab directory to path
addpath('../matlab');

% parse csv file
file = fopen(filename);
q    = fscanf(file, '%f, %f, %f, %f\n');
q    = reshape(q, 4, []);
fclose(file);

% display each quaternion
for i=1:size(q,2)
  plotState(q(:,i));
  title('display qorg');
  drawnow;
end

end