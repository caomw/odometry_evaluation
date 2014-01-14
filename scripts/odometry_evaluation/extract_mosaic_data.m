
arg_list = argv();

if nargin != 2
  error("wrong number of arguments!")
end

m = load(arg_list{1});
file = fopen(arg_list{2},'w');

n = size(m.Mosaic_Poses,2);
for i=1:n
  fprintf(file,'%f %f %f %f %f %f %f\n', m.Mosaic_timestamps(i),m.Mosaic_Poses(1:3,i),m.Mosaic_Poses(5:7,i))
end
fclose(file);
