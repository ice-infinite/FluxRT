function mc_plot(outdir,name,x,Y,labels,xname,yname,heading)
% 每个图单独窗口；保存后关闭，避免批量实验打开过多窗口。
f=figure('Visible','off','Position',[100 100 1000 580]);
plot(x,Y,'LineWidth',1.4); grid on; xlabel(xname); ylabel(yname);
title(heading,'Interpreter','none');
if strcmp(name,'L01_vector'),axis equal;end
if ~isempty(labels), legend(labels,'Location','best','Interpreter','none'); end
print(f,fullfile(outdir,[name '.png']),'-dpng','-r150');
close(f);
end
