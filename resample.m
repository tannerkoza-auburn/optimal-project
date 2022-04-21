function out = resample(W)

N = length(W); % number of particles
Q = cumsum(W); % cummulative sum
t = rand(1,N+1);
T = sort(t);
T(N+1) = 1;
i = 1; j = 1;

while i<=N
    if T(i) < Q(j)
        Index(i) = j;
        i = i + 1;
    else
        j = j + 1;
    end
end

out = Index;

end