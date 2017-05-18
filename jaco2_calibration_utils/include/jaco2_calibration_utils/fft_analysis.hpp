#ifndef FFT_ANALYSIS_HPP
#define FFT_ANALYSIS_HPP

#include <Eigen/SVD>
#include <Eigen/QR>
#include <unsupported/Eigen/FFT>

struct FFTAnalysis
{
    std::vector<double> getSineParams(Eigen::VectorXd& x, Eigen::VectorXd& y, std::size_t n_sine)
    {

        Eigen::FFT<double> fft;

        Eigen::VectorXd res = y;
        Eigen::VectorXcd g;
        Eigen::VectorXd freqs;
        freqs.setZero(n_sine);
        std::vector<std::size_t> peak_loc;
        Eigen::VectorXd params;

        std::size_t len = x.size();

        for(std::size_t iter = 0; iter < n_sine; ++iter){
            fft.fwd(g,res);
            removePeaks(peak_loc,g);
            // Get starting value for frequency using fft peak
            std::size_t i = 0;
            getMaxLocFirstHalf(g, i);
            peak_loc.push_back(i);
            double w = 2.0*M_PI*(i-1 )/(x(len -1) - x(0));
            freqs(iter) = w;
            // Compute Fourier terms using all frequencies we have so far
            Eigen::MatrixXd X(len, 2*(iter+1));

            for(std::size_t k = 0; k <= iter; ++ k){
                Eigen::VectorXd tmp = freqs(k) *x;
                X.block(0, 2*k  , len, 1) = tmp.array().sin();
                X.block(0, 2*k+1, len, 1) = tmp.array().cos();
            }

            // Fit these terms to get the non-frequency starting values
            Eigen::JacobiSVD<Eigen::MatrixXd> svd(X, Eigen::ComputeThinU | Eigen::ComputeThinV | Eigen::FullPivHouseholderQRPreconditioner);
            params = svd.solve(y);



            if(iter < n_sine){
                res = y - X*params;    // remove these components to get next frequency
            }
        }
        std::vector<double> result;
        for(std::size_t k = 0; k < n_sine; ++k){
            double fcos = params(2*k);
            double fsin = params(2*k+1);
            result.emplace_back(std::sqrt(fcos*fcos + fsin * fsin));
            result.emplace_back(freqs(k));
            result.emplace_back(atan2(fsin, fcos));
        }

        return result;


    }

    void removePeaks(std::vector<std::vector<std::complex<double>>::iterator > & peaks)
    {
        for(auto p : peaks){
            *p = std::complex<double>(0,0);
        }
    }

    void removePeaks(std::vector<std::size_t> & peaks, Eigen::VectorXcd& func)
    {
        for(auto p : peaks){
            func(p) = std::complex<double>(0,0);
        }
    }

    void getMaxLocFirstHalf(const Eigen::VectorXcd& g, std::size_t& i)
    {
        double max = -1;
        for(std::size_t iter = 0; iter < std::floor(g.size()/2) +1; ++iter){
            std::complex<double> c = g(iter);
            double abs = std::abs(c);
            if( abs > max){
                max = abs;
                i = iter;
            }
        }
    }

};

#endif // FFT_ANALYSIS_HPP
