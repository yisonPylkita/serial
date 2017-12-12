/* To run these tests you need to change the define below to the serial port 
 * with a loop back device attached.
 * 
 * Alternatively you could use an Arduino:
 
void setup()
{
 Serial.begin(115200);
}

void loop()
{
 while (Serial.available() > 0) {
   Serial.write(Serial.read());
 }
}
 
*/

#include <string>
#include "catch.hpp"
#include <serial/serial.h>

#if defined(__linux__)
#include <pty.h>
#include <unistd.h>
#else
#include <util.h>
#endif


using namespace serial;

class SerialFixture {
public:
    SerialFixture()
    {
        if (openpty(&master_fd, &slave_fd, name, NULL, NULL) == -1) {
            perror("openpty");
            exit(127);
        }

        REQUIRE(master_fd > 0);
        REQUIRE(slave_fd > 0);
        REQUIRE(std::string(name).length() > 0);

        port1.reset(new Serial(std::string(name), 115200, Timeout::simpleTimeout(250)));
    }

    ~SerialFixture()
    {
        port1->close();
    }

protected:
    std::unique_ptr<Serial> port1;
    int master_fd = 0;
    int slave_fd = 0;
    char name[100]{};
};

TEST_CASE_METHOD(SerialFixture, "read", "[serial]") {
    write(master_fd, "abc\n", 4);
    std::string r = port1->read(4);
    REQUIRE(r == std::string("abc\n"));
}

TEST_CASE_METHOD(SerialFixture, "write", "[serial]") {
    char buf[5] = "";
    port1->write("abc\n");
    read(master_fd, buf, 4);
    REQUIRE(std::string(buf, 4) == std::string("abc\n"));
}

TEST_CASE_METHOD(SerialFixture, "timeout", "[serial]") {
    // Timeout a read, returns an empty string
    std::string empty = port1->read();
    REQUIRE(empty == std::string(""));

    // Ensure that writing/reading still works after a timeout.
    write(master_fd, "abc\n", 4);
    std::string r = port1->read(4);
    REQUIRE(r == std::string("abc\n"));
}

TEST_CASE_METHOD(SerialFixture, "partial read", "[serial]") {
    // Write some data, but request more than was written.
    write(master_fd, "abc\n", 4);

    // Should timeout, but return what was in the buffer.
    std::string empty = port1->read(10);
    REQUIRE(empty == std::string("abc\n"));

    // Ensure that writing/reading still works after a timeout.
    write(master_fd, "abc\n", 4);
    std::string r = port1->read(4);
    REQUIRE(r == std::string("abc\n"));
}
