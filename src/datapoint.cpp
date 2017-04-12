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
    cout << "Sensor ID: " << static_cast<int>(this->data_type) << " (1:LIDAR | 2:RADAR | 3:STATE)" << endl;
    cout << "Raw Data: " << endl;
    cout << this->raw << endl;

  } else {
    cout << "Datapoint is not initialized" << endl;
  }
}

VectorXd DataPoint::get() const {
  return this->raw;
}