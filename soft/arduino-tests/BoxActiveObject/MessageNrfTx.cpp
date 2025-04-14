#include "MessageNrfTx.h"
#include <string.h>

MessageNrfTx::MessageNrfTx(const char* dataToSend) {
  strncpy(data_, dataToSend, sizeof(data_) - 1);
  data_[sizeof(data_) - 1] = '\0';
}