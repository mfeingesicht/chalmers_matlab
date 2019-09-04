function [Phi_Phi,Phi_F,Phi_R,A_e, B_e,C_e,F,Phi] = mpcgain(Ap,Bp,Cp,Nc,Np,r,K_bar)

% coder.extrinsic('size','eye','zeros');
% coder.varsize('m1',[]   );
m1=size(Cp,1);
[n1,n_in]=size(Bp);
A_e=eye(n1+m1,n1+m1);
A_e(1:n1,1:n1)=Ap;
A_e(n1+1:n1+m1,1:n1)=Cp*Ap;
B_e=zeros(n1+m1,n_in);
B_e(1:n1,:)=Bp;
B_e(n1+1:n1+m1,:)=Cp*Bp;
C_e=zeros(m1,n1+m1);
C_e(:,n1+1:n1+m1)=eye(m1,m1);
[wid,~] = size(C_e);
h = zeros(3*Np,6);
F = zeros(3*Np,6);
h(1:wid,:)=C_e;
F(1:wid,:)=C_e*A_e;

for kk=2:Np
h((kk-1)*wid+1:kk*wid,:)=h((kk-2)*wid+1:(kk-1)*wid,:)*A_e;
F((kk-1)*wid+1:kk*wid,:)= F((kk-2)*wid+1:(kk-1)*wid,:)*A_e;
end

v=h*B_e;

len1 = size(B_e,2);
Phi=zeros(Np*wid,Nc*len1); %declare the dimension of Phi

Phi(:,1:len1)=v; % first column of Phi
% len1 = wid; % the length for CB,CAB,...
for i=2:Nc
    Phi(:,(i-1)*len1+1:i*len1)=[zeros((i-1)*wid,len1);v(1:(Np-i+1)*wid,1:len1)]; %Toeplitz matrix
end

BarRs = zeros(Np*wid,1);

    for i = 1:Np
        BarRs((i-1)*wid+1:i*wid,1) = r;
    end

% BarRs
Phi_Phi = Phi.'*K_bar*Phi;
Phi_Phi = (Phi_Phi.' + Phi_Phi)/2;
% F
Phi_F= Phi'*K_bar*F;
Phi_R=Phi.'*K_bar*BarRs;


end

