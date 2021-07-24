function [out] = sym2str(symbol)

    symbol = sym(symbol); %insure input is symbolic
    siz = prod(size(symbol));   %find the number of elements in "sy"
    for i = 1:siz   %dump it into a cell array with the same number of elements
        in{i} = char(symbol(i)); %convert to char
    end
    if siz == 1
        in = char(in);      %revert back to a 'char' array for single answers
    end
    out = in;