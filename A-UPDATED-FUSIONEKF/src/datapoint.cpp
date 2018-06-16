#include "datapoint.h"

using namespace std;

DataPoint::DataPoint(){
  _initialized = false;
}

DataPoint::DataPoint(const long long timestamp, const DataPointType data_type, const VectorXd raw){
  this->set(timestamp, data_type, raw);
}

void DataPoint::set(const long timestamp, const DataPointType data_type, const VectorXd raw){
  _timestamp = timestamp;
  _data_type = data_type;
  _raw = raw;
  _initialized = true;
}

VectorXd DataPoint::get_raw_data() const{
  return _raw;
}

VectorXd DataPoint::get_state() const{

  VectorXd state(4);

  if (_data_type == DataPointType::LIDAR){

    double x = _raw(0);
    double y = _raw(1);
    state << x, y, 0.0, 0.0;

  } else if (_data_type == DataPointType::RADAR){

    state = convert_polar_to_cartesian(_raw);
  }

  return state;
}

long long DataPoint::get_timestamp() const{
  return _timestamp;
}

DataPointType DataPoint::get_type() const{
  return _data_type;
}

void DataPoint::print() const{

  if (_initialized){
    cout << "Timestamp: " << _timestamp << endl;
    cout << "Sensor ID: " << static_cast<int>(_data_type) << " (LIDAR = 0 | RADAR = 1)" << endl;
    cout << "Raw Data: " << endl;
    cout << _raw << endl;
  } else {
    cout << "DataPoint is not initialized." << endl;
  }
}
