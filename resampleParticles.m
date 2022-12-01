%function particles= resampleParticles(particles, Nmin, fp_debug, resample_randstream)
function particles= resampleParticles(particles, Nmin) %, resample_randstream
%
% Resample particles if their weight variance is such that N-effective
% is less than Nmin.
%

N = length(particles);
w = [particles.w];
ws = sum(w); 
w  = w/ws;

Neff = 1 / sum(w .^ 2);


if Neff < Nmin
    %fprintf(fp_debug, 'Particles resampled, Neff = %d\n', Neff);
    %disp(['Particles resampled, Neff = ', Neff]);
    
    %[keep]    = stratified_resample(w, resample_randstream);%, resample_randstream
    [keep]    = stratified_resample(w);
    %fprintf(fp_debug, 'Keep particles: ');
    particles = particles(keep);
%     for i=1:N
%         particles(i).w= 1/N; 
%         fprintf(fp_debug, '%d ', keep(i));
%         
%     end
    %fprintf(fp_debug, '\n');
    %disp('Keep particles: ');disp(keep);
else
    for i=1:N
        particles(i).w= particles(i).w / ws; 
    end
end
