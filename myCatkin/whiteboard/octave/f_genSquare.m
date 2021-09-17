# gen square

function [x, y] = f_genSquare(xspan, yspan, n)
  xmin = 0;
  ymin = 0;
  xmax = xspan;
  ymax = yspan;
  n = max(n,2); % block dumb input 
  
  xstep_est = 2*xspan/((n+1)/2); % +1 insures we dont get around
  ystep_est = 2*yspan/((n+1)/2); % +1 insures we dont get around
  x = zeros(n,1);
  y = zeros(n,1);
  xi = 0;
  yi = 0;
  
  i = 1;
  while i < n
    xi = xi + xstep_est;  
    x(i) = xi;
    y(i) = yi;
    i = i + 1;
    if xi > xmax
      while i < n
        yi = yi + ystep_est;
        x(i) = xi;
        y(i) = yi;        
        i = i + 1;
        if yi > ymax
          while i < n
            xi = xi - xstep_est;
            x(i) = xi;
            y(i) = yi;            
            i = i + 1;
            if xi < 0
              while i < n
                yi = yi - ystep_est;
                x(i) = xi;
                y(i) = yi;                
                i = i + 1;
              end
            end
          end
        end
      end
    end
  end
endfunction
