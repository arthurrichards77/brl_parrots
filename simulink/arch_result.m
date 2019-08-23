new_name = ['ilc' sprintf('_%d',floor(clock)) '.mat']
copyfile('ilc.mat',['results/' new_name],'f')