
source "utils.m"

function T = compute_true(U)
    T=zeros(size(U,1),3);
	current_T=v2t(zeros(1,3));
	for i=1:size(U,1),
		u=U(i,1:3)';
		current_T*=v2t(u);
		T(i,1:3)=t2v(current_T)';
	end
end