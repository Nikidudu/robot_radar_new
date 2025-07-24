/*
 *  NUS Calibur Robotics
 *  SPI_driver.h
 *
 *  Created on: 9 Dec 2024
 *      Author: JL
 *      Edited by : ______
 */

#include "build/build.h"
#include "io_driver.h"
#include "spi.h"

// #ifdef BUILD_WITH_SPI

#include "stm32f4xx_hal.h"

#include "Thread.hpp"
#include <vector>

#define SPI_BUFFER_SIZE    2048
#define THREAD_STACK_SIZE  4096                // DON'T CHANGE IF NOT NECESSARY TO DO SO
#define NB_SPI_PORTS       3                   // CHANGE ONLY IF NEEDED

class SPIDriver: public IODriver, public Thread {
    public:
        SPIDriver(SPI_HandleTypeDef* hspi); // Constructor
        virtual ~SPIDriver(); // Destructor
        uint8_t* getBuffer();
        xSemaphoreHandle getSemaphore();
        uint8_t getSenderID(SPI_HandleTypeDef* hspi);

        void init();
        void loop();

        void receive(const std::function<void (uint8_t sender_id, uint8_t* buffer, uint32_t length)> &receiver) override;
        void transmit(uint8_t* buffer, uint32_t length) override;

        void receiveSPI(uint8_t sender_id, uint8_t* buffer, uint32_t length);
        SPI_HandleTypeDef* getHspi();
        SPIDriver* getInstance(SPI_HandleTypeDef* hspi);
    private:
        static std::vector<SPIDriver*> spi_driver_list;
        SPI_HandleTypeDef* _hspi;
        uint32_t last_dma_index;

        // Needed for the HAL_SPI_RxCpltCallback to access SPIDriver's class attributes (i.e. receiver_func & buffer)
        SPI_TypeDef* mapper[NB_SPI_PORTS] = {SPI1, SPI2, SPI3};

        uint8_t* buffer;
        std::function<void (uint8_t sender_id, uint8_t* buffer, uint32_t length)> receiver_func; // User-defined callback function

    	xSemaphoreHandle semaphore;
};

// #endif /* BUILD_WITH_SPI */