// Static buffer

class CharBuffer {
  private:
    static const int MAX_SIZE = 128;  // You can adjust as needed
    char buffer[MAX_SIZE];
    int length;

  public:
    // Constructor
    CharBuffer() : length(0) {
        buffer[0] = '\0';
    }

    // Append characters from a Stream (e.g., Serial)
    int appendFrom(Stream& stream) {
        int i = 0;
        while (stream.available() && length < MAX_SIZE - 1) {
            char c = (char)stream.read();
            buffer[length++] = c;
            buffer[length] = '\0';  // Always null-terminate
            i++;
        }
        return i;
    }

    // Get current buffer length
    int getLength() const { return length; }

    // Get current buffer content
    const char* get() const { return buffer; }

    // Clear the buffer
    void reset() {
        length = 0;
        buffer[0] = '\0';
    }

    // Check for newline character
    bool hasLine() const {
        for (int i = 0; i < length; ++i) {
            if (buffer[i] == '\n' || buffer[i] == '\r') return true;
        }
        return false;
    }

    // Extract a line and shift buffer
    const char* readLine() {
        static char line[MAX_SIZE];

        int i = 0;
        while (i < length && buffer[i] != '\n') {
            line[i] = buffer[i];
            i++;
        }

        if (i < length && buffer[i] == '\n') {
            line[i] = '\n';
            i++;
        } else {
            return nullptr;
        }

        line[i] = '\0';

        int remaining = length - i;
        for (int j = 0; j < remaining; ++j) {
            buffer[j] = buffer[i + j];
        }
        length = remaining;
        buffer[length] = '\0';

        return line;
    }

    // Return human-readable status
    const char* getStatus() const {
        static char status[64];
        snprintf(
            status, sizeof(status),
            "Len: %d, Free: %d, HasLine: %s",
            length,
            MAX_SIZE - 1 - length,
            hasLine() ? "Yes" : "No"
        );
        return status;
    }
};
