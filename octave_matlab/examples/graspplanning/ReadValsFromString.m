function [vals, rem] = ReadValsFromString(str,N)

vals = zeros(N,1);
rem = str;
for j = 1:N
    [tok, rem] = strtok(rem, ' ');
    vals(j) = str2num(tok);
end
