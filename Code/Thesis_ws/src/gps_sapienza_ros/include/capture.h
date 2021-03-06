// Public interface for video capture
int initVideoCapture();
int  initCapture();
void closeCapture();
IplImage * nextFrame();
void printResults(float roll[], float yaw[], float pitch[]);
void writeVideo( IplImage* frame);
int NextFrame(IplImage** frame);