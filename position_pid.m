%PID Controller,位置式PID控制算法，u(k)对应执行机构的实际位置
 
clear all;
close all;
 
ts=0.001;
sys=tf(5.235e005,[1,87.35,1.047e004,0]); %transfer function (TF) 
dsys=c2d(sys,ts,'zoh');%Convert a model from continuous time to discrete time
%Index里没有，在Search里找
[num,den]=tfdata(dsys,'v');%Quick access to transfer function data
 
u_1=0.0;u_2=0.0;u_3=0.0;
y_1=0.0;y_2=0.0;y_3=0.0;
x=[0,0,0]';
error_1=0;
for k=1:1:1500
    time(k)=k*ts;
        
    S=3;
    if S==1
        kp=0.50;ki=0.001;kd=0.001;          
        rin(k)=1;                            %Step Signal
    elseif S==2
        kp=0.50;ki=0.001;kd=0.001;          
        rin(k)=sign(sin(2*2*pi*k*ts));  %Square Wave Signal
    elseif S==3
        kp=1.5;ki=1.0;kd=0.01;            %Sine Signal
        rin(k)=0.5*sin(2*2*pi*k*ts);           
    end
     
    u(k)=kp*x(1)+kd*x(2)+ki*x(3);   %PID Controller
    %Restricting the output of controller
    if u(k)>=10       
       u(k)=10;
    end
    if u(k)<=-10
       u(k)=-10;
    end
    %Linear model
    yout(k)=-den(2)*y_1-den(3)*y_2-den(4)*y_3+num(2)*u_1+num(3)*u_2+num(4)*u_3;%差分方程，迭代解法，(while微分方程是ode)
     
    error(k)=rin(k)-yout(k);
     
    %Return of parameters
    u_3=u_2;u_2=u_1;u_1=u(k);
    y_3=y_2;y_2=y_1;y_1=yout(k);
        
    x(1)=error(k);                 %Calculating P
    x(2)=(error(k)-error_1)/ts;  %Calculating D
    x(3)=x(3)+error(k)*ts;        %Calculating I
     
    error_1=error(k);
end
figure(1);
plot(time,rin,'k',time,yout,'k');
xlabel('time(s)'),ylabel('rin,yout');