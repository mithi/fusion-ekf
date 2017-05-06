#include "datapoint.h"

using namespace std;

DataPoint::DataPoint(){
  this->initialized = false;
}

DataPoint::DataPoint(const long long timestamp, const DataPointType data_type, const VectorXd raw){
  this->set(timestamp, data_type, raw);
}

void DataPoint::set(const long timestamp, const DataPointType data_type, const VectorXd raw){
  this->timestamp = timestamp;
  this->data_type = data_type;
  this->raw = raw;
  this->initialized = true;
}

VectorXd DataPoint::get() const{
  return this->raw;
}

VectorXd DataPoint::get_state() const{

  VectorXd state(4);

  if (this->data_type == DataPointType::LIDAR){

    double x = this->raw(0);
    double y = this->raw(1);
    state << x, y, 0.0, 0.0;

  } else if (this->data_type == DataPointType::RADAR){

    state = convert_polar_to_cartesian(this->raw);

  } else if (this->data_type == DataPointType::STATE){

    state = this->raw;
  }

  return state;
}

long long DataPoint::get_timestamp() const{
  return this->timestamp;
}

DataPointType DataPoint::get_type() const{
  return this->data_type;
}

void DataPoint::print() const{

  if (this->initialized){

    cout << "Timestamp: " << this->timestamp << endl;
    cout << "Sensor ID: " << static_cast<int>(this->data_type) << " (LIDAR = 0 | RADAR = 1 | STATE = 2)" << endl;
    cout << "Raw Data: " << endl;
    cout << this->raw << endl;

  } else {

    cout << "DataPoint is not initialized." << endl;
  }
}
