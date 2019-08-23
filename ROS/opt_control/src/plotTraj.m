%plot
clear all;
close all;
numberOfEx = 1;

for(i=1:1:numberOfEx)
	eval(["data",num2str(i)]);
	fi=0;
			
	figure(++fi);
	subplot(numberOfEx,1,i);
	plot(X(1,:), X(2,:));
	hold on;
	plot(wpX(1,:), wpX(2,:), "r.");
	xlabel('x (m)');
	ylabel('y (m)');
		
	figure(++fi);
	subplot(numberOfEx,1,i);
	plot(T, X(1,:), 'r', T,X(2,:), 'g');
	hold on;
	plot(wpT(1,:), wpX(1,:), 'r*', wpT(2,:), wpX(2,:), 'g*');
	xlabel('t (s)');
	ylabel('pos (m)');
	legend('posx', 'posy');
	
	figure(++fi);
	subplot(numberOfEx,1,i);
	plot(T, dX(1,:), 'r', T,dX(2,:), 'g');
	hold on;
	plot(wpT(1,:), wpdX(1,:), 'r*', wpT(2,:), wpdX(2,:), 'g*');
	xlabel('t (s)');
	ylabel('vel (m)');
	legend('velx', 'vely');
	
	figure(++fi);
	subplot(numberOfEx,1,i);
	plot(T, ddX(1,:), 'r', T,ddX(2,:), 'g');
	hold on;
	plot(wpT(1,:), wpddX(1,:), 'r*', wpT(2,:), wpddX(2,:), 'g*');
	xlabel('t (s)');
	ylabel('acc (m)');
	legend('accx', 'accy');
	
	figure(++fi);
	subplot(numberOfEx,1,i);
	h = plot(T, dddX(1,:), 'r', T,dddX(2,:), 'g');
	xlabel('t (s)');
	ylabel('jerk (m)');
	legend('jerkx', 'jerky');
endfor

waitfor(h)
disp('exit')

