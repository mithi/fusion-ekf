#include "datapoint.h"

DataPoint::DataPoint(){ 
  this->is_initialized = false; 
}

DataPoint:: DataPoint(const long timestamp, const DataPointType data_type, const VectorXd raw){
  this->set(timestamp, data_type, raw);
}

void DataPoint::set(const long timestamp, const DataPointType data_type, const VectorXd raw){
  this->timestamp = timestamp;
  this->data_type = data_type;
  this->raw = raw;
  this->is_initialized = true; 
}

void DataPoint::print() const{

  if (this->is_initialized){

    cout << "Timestamp: " << this->timestamp << endl;
    cout << "Sensor ID: " << static_cast<int>(this->data_type) << " (LIDAR = 0 | RADAR = 1 | STATE = 2)" << endl;
    cout << "Raw Data: " << endl;
    cout << this->raw << endl;

  } else {
    cout << "DataPoint is not initialized." << endl;
  }
}

VectorXd DataPoint::get() const {
  return this->raw;
}