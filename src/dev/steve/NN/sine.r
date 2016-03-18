set.seed(1405)
x <- sort(10*runif(50))
y <- sin(x) + 0.2*rnorm(x)

library(nnet)
nn <- nnet(x, y, size=6, maxit=40, linout=TRUE)
jpeg('rplot.jpg')
plot(x, y)
plot(sin, 0, 10, add=TRUE)
x1 <- seq(0, 10, by=0.1)
lines(x1, predict(nn, data.frame(x=x1)), col="green")

dev.off()
