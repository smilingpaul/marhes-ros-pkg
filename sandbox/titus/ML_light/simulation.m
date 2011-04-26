set(0, 'defaultfigurevisible', 'off');
[x,y]=meshgrid([0:0.01:1],[0:0.01:1]);
 
for t=[0:1000]
    z=x-x; %Initialize empty z grid
    for nx=[1:10]
        for ny=[1:10]
          coeff=(1-(-1)^nx)*(1-(-1)^ny)/(nx^3*ny^3);
          z=z+coeff*sin(nx*pi.*x).*sin(ny*pi.*y)*cos(sqrt(nx^2+ny^2)*pi*t/100);
        end
    end
 
    surf(x,y,z);
    axis([0 1 0 1 -4 4]);
    shading interp;
    xlabel("x");
    ylabel("y");
    zlabel("u(x,y)");
    filename=sprintf('output/%05d.png',t);
    print(filename);
end