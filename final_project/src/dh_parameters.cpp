#include <iostream>
#include <vector>
#include <cmath>

class DhParameters
{
    public:
        void setDhParameters(const std::vector<float> a, const std::vector<float> d,
                const std::vector<float> theta, const std::vector<float> alpha)
        {
            if (a_.size() != d_.size() ||
                    d_.size() != theta_.size() ||
                    theta_.size() != alpha_.size())
            {
                throw std::invalid_argument("Denavit Hartenberg parameter "
                        "vectors cannot have different sizes");
            }
            a_ = a;
            d_ = d;
            theta_ = theta;
            alpha_ = alpha;
            jointNum_ = a_.size();
        }

        std::vector<float> getA() const
        {
            return a_;
        }

        std::vector<float> getD() const
        {
            return d_;
        }

        std::vector<float> getTheta() const
        {
            return theta_;
        }
        std::vector<float> getAlpha() const
        {
            return alpha_;
        }

        uint getNumberOfJoints() 
        {
            return jointNum_;
        }

    private:
        std::vector<float> a_;
        std::vector<float> d_;
        std::vector<float> theta_;
        std::vector<float> alpha_;

        uint jointNum_;

        std::vector<float> getVectorfInRadians(std::vector<float> vector) const 
        {
            std::vector<float> vectorInRadians = vector;
            for (auto& element : vectorInRadians)
            {
                element = element*M_PI/180;
            }
            return vectorInRadians;
        }
};
