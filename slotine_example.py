# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
# % Daniel Wiese
# % Example from Slotine, Li - Applied Nonlinear Control
# % Bounded function with unbounded derivative
# % 2012-Jul-19
# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
import numpy as np
import math
import matplotlib.pyplot as plt

# %Input parameters
white = [1, 1, 1]
black = [0, 0, 0]
init = 0
fin = 10
t = np.linspace(init, fin, 10000)

# %lower left corner
llcorn = [8, -0.03]
width = 0.5
height = 0.06
wheretozoom=[llcorn, width, height]
wheretoputbox=[0.6, 0.6, 0.25, 0.25]

# %The function
# f = np.array(len(t))
f = []
# print(len(t))
for i in range(0, len(t)):
    # print(i)
    # f[i] = math.exp(-t[i]*math.sin(math.exp(2*t[i])))
    f.append(math.exp(-t[i]*math.sin(math.exp(2*t[i]))))

# %Plot the function
fig = plt.figure()
plt.plot(t, f)
plt.show()
# set(gcf,'DefaultLineLineWidth',1)
# set(gcf,'DefaultlineMarkerSize',6)
# plt.plot(t,f,'Color',black,'LineStyle','-')
# plt.set_title('Slotine Example Plot','interpreter','latex','FontSize',10)
# xlabel('$t$','interpreter','latex','FontSize',10)
# ylabel('$f(t)$','interpreter','latex','FontSize',10)
# plt.paxes.hand{1}=gca;
# # %Dont really need this stuff
#     plt.pos=get(plt.paxes.hand{1},'Position');
#     plt.ylimit=get(plt.paxes.hand{1},'YLim');
#     set(plt.paxes.hand{1},'YLim',[-1 1])
# grid on
# # %Now plot the zoomed rectangle
#     hold on
#     source_rect = rectangle('Position',wheretozoom);
#     plt.zaxes.hand{1}=axes('position',wheretoputbox);
#     plot(t,f,'Color',col.black,'LineStyle','-')
#     xlim([llcorn(1) llcorn(1)+width])
#     ylim([-0.001 0.001])
#     set(source_rect,'linestyle','-','Edgecolor', 'k')
#     set(plt.zaxes.hand{1},'xtick', [], 'ytick', [])
#     box on;
# uistack(plt.paxes.hand{1},'bottom')
# set(gcf,'Units','pixels');
# set(gcf, 'OuterPosition', [200, 200, 700, 500]) %1520x980
# set(gcf,'PaperUnits','inches','PaperPosition',[0 0 6 5]);
# set(gcf,'PaperPositionMode','manual')
# set(gcf,'InvertHardCopy','off')
# set(gcf,'color',col.white)
