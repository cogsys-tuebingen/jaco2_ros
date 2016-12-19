#ifndef JACO2RESIDUALVECTOR_H
#define JACO2RESIDUALVECTOR_H
class Jaco2ResidualVector
{
public:
    Jaco2ResidualVector();
    Jaco2ResidualVector(const std::string& robot_model, const std::string& chain_root, const std::string& chain_tip);
};

#endif // JACO2RESIDUALVECTOR_H
