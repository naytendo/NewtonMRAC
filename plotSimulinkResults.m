function plotSimulinkResults(data,params)
    
    Ns = data.Ns;
    trackingError = data.trackingError;
    totTime = data.totTime;
    triggerTimes = data.triggerTimes;
    xTot = data.xTot;
    condK = data.condK;
    condN = data.condN;
    gCenters = data.gCenters;
    Epsilons = data.Epsilons;
    runTimes = data.runTimes;
    triggerErrors = data.triggerErrors;


    B = params.B;
    P = params.P;
    rmax = params.rmax;
    set(groot, 'DefaultFigurePosition', [300, 100, 500, 400])
    
    fSS = figure();
    
    
    
    %% Here we plot the results
    
    
    subplot(3,1,1:2)
    hold on
    trackingErrorNorm = sqrt(sum(trackingError.^2,2));
    semilogy(totTime,trackingErrorNorm./norm(B'*P,2),'linewidth',1.5); % Scaling of tracking error is applied as Y axis of graph indicates
    
    
    
    
    if ~isempty(triggerTimes)
        tm = [totTime(2);triggerTimes];
        line([tm(1),runTimes],[Epsilons(end), Epsilons(end)],'linestyle','--','linewidth',2.5,'color',[0.9290 0.6940 0.1250])
        l = semilogy(triggerTimes(1:end),triggerErrors(1:end),'o','markerSize',5);
        l.MarkerFaceColor = l.Color;
        line([tm(end),runTimes],[Epsilons(end), Epsilons(end)],'linestyle','-.','linewidth',1.5,'color','k')
        for ll = 1:length(triggerTimes)
            line([tm(ll),tm(ll+1)],[Epsilons(ll), Epsilons(ll)],'linestyle','-.','linewidth',1.5,'color','k')
            line([tm(ll+1),tm(ll+1)],[Epsilons(ll), Epsilons(ll+1)],'linestyle','-.','linewidth',1.5,'color','k')
        end
    end
    
    
    set(gca,'Yscale','log')
    set(gca,'fontsize',10)
    ylabel('$\frac{\|\tilde{x}_N\|}{\|B^TP\|}$','interpreter','latex','fontsize',12)
    
    legend('tracking error','ultimate error bound','trigger events at $t_m$','$\sup_{\xi \in \Omega_{m}} P_{H_{N}}(\xi)$','interpreter','latex','fontsize',12,'Location','SouthEast')
    grid on
    subplot(3,1,3)
    plot([tm;runTimes],[Ns;Ns(end)],'linewidth',1.5)
    xlabel('time (s)','interpreter','latex','fontsize',12)
    ylabel('$N$','interpreter','latex','fontsize',12)
    ylim([100 500]) % Changed the Y range due to new Nmax
    set(gca,'Yscale','log')
    grid on
    set(gca,'fontsize',10)
    
    figure();
    % plot(Omega(:,1),Omega(:,2),'k.','markersize',1)
    hold on
    plot(gCenters(:,1),gCenters(:,2),'ko')
    plot(xTot(:,1),xTot(:,2),'b-.','linewidth',2)
    grid on
    theta = linspace(0,360,100);
    plot(rmax*sind(theta),rmax*cosd(theta),'color',[0.4940 0.1840 0.5560],'linewidth',5)
    xlabel('$x_1$','interpreter','latex','fontsize',12)
    ylabel('$x_2$','interpreter','latex','fontsize',12)
    set(gca,'fontsize',10)
    
    index = find(totTime == triggerTimes(end-1));
    plot(xTot(index:end,1),xTot(index:end,2),'color',[0.9290 0.6940 0.1250],'linewidth',1.5)
    legend('$\Xi_N$','$x$','$x_r$','$x_{ss}$','fontsize',12,'interpreter','latex')
    
    %%
    figure()
    loglog(Ns,condK,'-o','LineWidth',2);
    hold on
    loglog(Ns,condN,'-d','LineWidth',2);
    grid on
    set(gca,'fontsize',10)
    xlabel('$\log(N)$','interpreter','latex','fontsize',12)
    ylabel('$\log$(Condition Number)','interpreter','latex','fontsize',12)
    legend('Standard Basis','Newton Basis','fontsize',12,'interpreter','latex','Location','NorthWest')
end