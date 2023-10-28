#ifndef KEY_VALUE_READER_H
#define KEY_VALUE_READER_H

#include <SD.h>
#include "FS.h"

class KeyValueReader
{
private:
    File file;
    char line[128];

protected:
  char *fgets(char *buffer, int size);
  void skipEmptyAndCommentLines();

public:
    KeyValueReader(const char *filename);
    void process(void (*callback)(const char *, const char *));
    void close();
};

#endif