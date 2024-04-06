#include "sensors/mq3.hpp"

namespace sensors {

MQ3::MQ3(String board, float voltage_resolution, uint8_t adc_bit_resolution, uint8_t pin) {
    this->core = new ::MQUnifiedsensor(board, voltage_resolution, (int)adc_bit_resolution, (int)pin, this->mq_type);
    pinMode(pin, INPUT);  // Analog
}

void MQ3::begin() {
    Serial.println("========== MQ3 Sensor =============");

    this->core->setRegressionMethod(1);

    this->core->setA(this->a_regression_coeff);
    this->core->setB(this->b_regression_coeff);
    this->core->init();
    this->core->setRL(this->RL);

    // this->core->serialDebug(true);  // Uncomment if you want to print the table on the serial port

    Serial.println("Calibrating MQ3...");
    this->calibrate();
    Serial.println("Calibrating Done.");
    Serial.println("=================================");
}

float MQ3::calibrate() {
    float calcR0 = 0;
    for (size_t i = 0; i < 10; i++) {
        this->core->update();
        calcR0 += this->core->calibrate(this->ratio_clean_air);
    }
    Serial.println(calcR0);
    this->core->setR0(calcR0 / 10);

    if (isinf(calcR0)) {
        Serial.println("[WARNING]: Connection issue, R0 is inifinite (Open circuit detected), please check your wiring and supply");
        // while (1)
        //     ;
    }
    if (calcR0 == 0) {
        Serial.println("[WARNING]: Connection issue found, R0 is zero (Analog pin shorts to ground), please check your wiring and supply");
    }

    return calcR0;
}

float MQ3::read() {
    this->core->update();
    return this->core->readSensor();  // ppm value
}

}  // namespace sensors
