# return path length of input x,y vector

function pathLen = f_getPathlen(x,y)
  nx = length(x);
  ny = length(y);
  
  pathLen = 0;
  if nx ~= ny
    pathLen = -1;
  else
    i = 1;
    while i < nx
      i = i + 1;
      seg = ((x(i)-x(i-1))^2 + (y(i)-y(i-1))^2)^0.5;
      pathLen = pathLen + seg;
    endwhile
  endif
endfunction
