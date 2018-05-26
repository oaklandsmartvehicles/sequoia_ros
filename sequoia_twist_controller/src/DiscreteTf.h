#ifndef DISCRETETF_H_
#define DISCRETETF_H_

#include <vector>

typedef std::vector<double> TfPolynomial;
typedef std::vector<double> TfValueBuffer;

float steering_cmd_filter;

class DiscreteTf
{

public:
  DiscreteTf(double ts) : ts_(ts) {reset();}

  void setSampleTime(double ts) {
    ts_ = ts;
  }

  double getSampleTime() {
    return ts_;
  }

  void addRealPole(double a){
    TfPolynomial new_num(2);
    new_num[0] = new_num[1] = 1.0 / (a + 2.0 / ts_);

    TfPolynomial new_den(2);
    new_den[0] = (a - 2.0 / ts_) / (a + 2.0 / ts_);
    new_den[1] = 1.0;

    num_polynomials_.push_back(new_num);
    den_polynomials_.push_back(new_den);
  }

  void addRealZero(double b){
    TfPolynomial new_num(2);
    new_num[0] = b - 2.0/ts_;
    new_num[1] = b + 2.0/ts_;

    TfPolynomial new_den(2);
    new_den[0] = 1.0;
    new_den[1] = 1.0;

    num_polynomials_.push_back(new_num);
    den_polynomials_.push_back(new_den);
  }

  void addComplexPole(double sigma, double omega){
    double temp = 4.0 / ts_ / ts_ + 4.0 * sigma / ts_ + sigma * sigma + omega * omega;

    TfPolynomial new_num(3);
    new_num[0] = 1.0 / temp;
    new_num[1] = 2.0 / temp;
    new_num[2] = 1.0 / temp;

    TfPolynomial new_den(3);
    new_den[0] = 1.0 - 8.0 * sigma / ts_ / temp;
    new_den[1] = (2.0 * sigma * sigma + 2.0 * omega * omega - 8.0 / ts_ / ts_) / temp;
    new_den[2] = 1.0;

    num_polynomials_.push_back(new_num);
    den_polynomials_.push_back(new_den);
  }

  void addComplexZero(double sigma, double omega){
    TfPolynomial new_num(3);
    new_num[0] = sigma * sigma + omega * omega - 4.0 * sigma / ts_ + 4.0 / ts_ / ts_;
    new_num[1] = 2.0 * sigma * sigma + 2.0 * omega * omega - 8.0 / ts_ / ts_;
    new_num[2] = 4.0 / ts_ / ts_ + 4.0 * sigma / ts_ + sigma * sigma + omega * omega;

    TfPolynomial new_den(3);
    new_den[0] = 1.0;
    new_den[1] = 2.0;
    new_den[2] = 1.0;

    num_polynomials_.push_back(new_num);
    den_polynomials_.push_back(new_den);
  }

  void setGain(double gain){
    gain_ = gain;
  }

  void computeTaps(){
    for (unsigned int i=0; i<den_polynomials_.size(); i++){
      a_ = polyMult(a_, den_polynomials_[i]);
    }
    for (unsigned int i=0; i<num_polynomials_.size(); i++){
      b_ = polyMult(b_, num_polynomials_[i]);
    }
    input_hist_.resize(b_.size());
    output_hist_.resize(a_.size());
  }

  double iterateTf(double input){
    if (input_hist_.size() == 0){
      return 0.0;
    }

    bufferPush(input, input_hist_);
    double output = 0.0;
    for (unsigned int i=0; i<input_hist_.size(); i++){
      output += b_[i] * input_hist_[i];
    }
    for (unsigned int i=0; i<output_hist_.size()-1; i++){
      output -= a_[i] * output_hist_[i+1];
    }
    bufferPush(output, output_hist_);
    return gain_ * output;
  }

  void reset(){
    a_.clear();
    b_.clear();
    gain_ = 1.0;
    num_polynomials_.clear();
    den_polynomials_.clear();
    input_hist_.clear();
    output_hist_.clear();
  }

  void clearHist(){
    for (unsigned int i=0; i<input_hist_.size(); i++){
      input_hist_[i] = 0.0;
    }

    for (unsigned int i=0; i<output_hist_.size(); i++){
      output_hist_[i] = 0.0;
    }
  }

private:

  TfPolynomial polyMult(TfPolynomial p1, TfPolynomial p2){

    if (p1.size() == 0){
      return p2;
    }

    if (p2.size() == 0){
      return p1;
    }

    TfPolynomial result(p1.size() + p2.size() - 1, 0.0);
    for (unsigned int i = 0; i < p1.size(); i++) {
      for (unsigned int j = 0; j < p2.size(); j++) {
        result[i+j] += p1[i] * p2[j];
      }
    }
    return result;
  }

  void bufferPush(double in_val, TfValueBuffer& buffer){
    buffer.erase(buffer.begin());
    buffer.push_back(in_val);
  }

  double ts_;

  std::vector<TfPolynomial> num_polynomials_;
  std::vector<TfPolynomial> den_polynomials_;
  double gain_;

  // Discrete taps
  TfPolynomial a_;
  TfPolynomial b_;

  // Time history of inputs and outputs
  TfValueBuffer input_hist_;
  TfValueBuffer output_hist_;

};

#endif // DISCRETETF_H_
