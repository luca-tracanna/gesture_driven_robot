#include <opencv2/opencv.hpp>

#include <AprilTags/TagDetector.h>
#include <AprilTags/TagDetection.h>
#include <AprilTags/Tag36h11.h>

#include <mqtt/client.h>

#include <libcamera/libcamera.h>
#include "image.h"
#include "event_loop.h"

#include <vector>
#include <cmath>
#include <csignal>
#include <boost/json.hpp>
#include <iostream>


#ifndef PI
constexpr double PI = 3.14159265358979323846;
#endif

using namespace libcamera;
using namespace std::chrono_literals;

static constexpr double width = 800;
static constexpr double height = 600;
static constexpr double tagSize =  0.053;
static constexpr double height_from_ground = 0.325;
static constexpr double rad_coeff = 180/PI;


static AprilTags::TagDetector detector{AprilTags::tagCodes36h11};

static std::string handleTags(const cv::Mat& image){


	// Taglio la parte superiore dell'immagine perché non utile per il detection
	cv::Rect roi(0, 150, width, height-150);
	cv::Mat image_cropped = image(roi);

	cv::resize(image_cropped, image_cropped, cv::Size(), 0.5, 0.5);

	// Eseguo uno sharpening per migliorare la qualità dei contorni
	
	cv::Mat filter = (cv::Mat_<double>(3,3) << -0.5, -0.5,-0.5,-0.5,5,-0.5,-0.5,-0.5,-0.5);
	
	cv::filter2D(image_cropped, image_cropped, -1, filter);
	
	// Converto l'immagine in scala di grigi
	cv::Mat image_gray{};
	cv::cvtColor(image_cropped, image_gray, cv::COLOR_BGR2GRAY);

	// Eseguo un thresholding binario sull'immagine 
	cv::Mat thresholded_image_gray;
	cv::threshold(image_gray, thresholded_image_gray, 110, 255, cv::THRESH_BINARY);
	
	// Eseguo la detection sull'immagine binaria
	auto detections = detector.extractTags(thresholded_image_gray);


	// Se non ho trovato tag restituisco un tag vuoto
	if(detections.size() == 0) {
		// Se non trovo tag, stampo l'immagine senza linee
		std::thread t([thresholded_image_gray](){ cv::imwrite("result.png", thresholded_image_gray); });
		t.detach();

		return boost::json::serialize(boost::json::value{
			{"ID", -1},
			{"dist", 0},
			{"yaw", 0},
			{"phi", 0},
		});
	}
	
	
	// Per il calcolo della posizione mi basta solo il primo tag che vedo. 
	const AprilTags::TagDetection& detection = detections[0];

	cv::Mat colored_with_lines_on_tag;
	cv::cvtColor(thresholded_image_gray, colored_with_lines_on_tag, cv::COLOR_GRAY2BGR);
	cv::line(colored_with_lines_on_tag, cv::Point2f{detection.p[0].first, detection.p[0].second }, cv::Point2f{detection.p[1].first, detection.p[1].second },{0,255,0}, 2);
	cv::line(colored_with_lines_on_tag, cv::Point2f{detection.p[1].first, detection.p[1].second }, cv::Point2f{detection.p[2].first, detection.p[2].second },{0,255,0}, 2);
	cv::line(colored_with_lines_on_tag, cv::Point2f{detection.p[2].first, detection.p[2].second }, cv::Point2f{detection.p[3].first, detection.p[3].second },{0,255,0}, 2);
	cv::line(colored_with_lines_on_tag, cv::Point2f{detection.p[3].first, detection.p[3].second }, cv::Point2f{detection.p[0].first, detection.p[0].second },{0,255,0}, 2);

	// Stampo l'immagine binaria
	std::thread t([colored_with_lines_on_tag](){ cv::imwrite("result.png", colored_with_lines_on_tag); });
	t.detach();
	
	
	Eigen::Vector3d translation{};
	Eigen::Matrix3d rotation{};

	int width = thresholded_image_gray.size().width;
	int height = thresholded_image_gray.size().height;
	double f = (width/2) / 0.6032; //tan(31.1°);
	

	detection.getRelativeTranslationRotation(tagSize, f, f, width / 2, height / 2, translation, rotation);

	
	// phi è l'angolo tra il centro della telecamera e il punto in cui si trova il marker nell'immagine 
	const double phi = std::atan2(translation(1), translation(0)) * rad_coeff; // in gradi

	// calcolo la distanza del tag
	const double norm = translation.norm();
	const double dist = std::sqrt(norm*norm - height_from_ground*height_from_ground);

	
	// determino le coordinate minime e massime del tag
	float min_x = std::min({detection.p[0].first, detection.p[1].first, detection.p[2].first, detection.p[3].first});
	float max_x = std::max({detection.p[0].first, detection.p[1].first, detection.p[2].first, detection.p[3].first});
	float min_y = std::min({detection.p[0].second, detection.p[1].second, detection.p[2].second, detection.p[3].second});
	float max_y = std::max({detection.p[0].second, detection.p[1].second, detection.p[2].second, detection.p[3].second});


	float box_height = max_y - min_y;
	float box_width = max_x - min_x;

	// bottom_sx, bottom_dx, top_dx, top_sx 
	std::array<cv::Point2f, 4> old_values{
		cv::Point2f{detection.p[0].first - min_x, detection.p[0].second - min_y},
		cv::Point2f{detection.p[1].first - min_x, detection.p[1].second - min_y},
		cv::Point2f{detection.p[2].first - min_x, detection.p[2].second - min_y}, 
		cv::Point2f{detection.p[3].first - min_x, detection.p[3].second - min_y}
	};

	int point_on_left_side = 0;

	if(old_values[1].x == 0)
		point_on_left_side = 1;
	else if(old_values[2].x == 0)
		point_on_left_side = 2;
	else if(old_values[3].x == 0)
		point_on_left_side = 3;


	std::array<cv::Point2f, 4> new_values;

	int point_on_bottom_side = (point_on_left_side +1) %4;
	int point_on_right_side = (point_on_left_side +2) %4;
	int point_on_top_side = (point_on_left_side +3) %4;

	new_values[point_on_top_side] = old_values[point_on_top_side];
	new_values[point_on_bottom_side] = cv::Point2f(old_values[point_on_bottom_side].x, box_width);
	new_values[point_on_right_side] = cv::Point2f(old_values[point_on_right_side].x, box_width * old_values[point_on_right_side].y / box_height);
	new_values[point_on_left_side] = cv::Point2f(old_values[point_on_left_side].x, box_width * old_values[point_on_left_side].y / box_height);


	// top_dx - bottom_dx
	cv::Point2f vettore1 = new_values[2] - new_values[1];
	
	//angolo compreso tra il vettore e l'asse y arrotondato a 3 cifre decimale
    double angolo_gradi = std::round(std::acos(-vettore1.y / std::sqrt(vettore1.x * vettore1.x + vettore1.y * vettore1.y)) * rad_coeff * 100)/100;
    


	int yaw_sign = new_values[2].x > new_values[1].x ? 1 : -1;

	double yaw = yaw_sign * angolo_gradi;


	boost::json::value json_tag = {
		{"ID", detection.id},
		{"dist", dist},
		{"yaw", yaw},
		{"phi", phi},
	};
	
	// Pubblico il messaggio creato
	return boost::json::serialize(json_tag);

}



static const std::string CLIENT_ID { "tag_module" };
const std::string TAGS_TOPIC { "/tags" };
std::unique_ptr<mqtt::client> client;

static std::shared_ptr<Camera> camera;

static void processRequest(Request *request)
{
	if (request->status() == Request::RequestCancelled)
		return;
	
	const auto bufferPair = request->buffers().begin();

	FrameBuffer *buffer = bufferPair->second;
	const StreamConfiguration &streamConfig = bufferPair->first->configuration();

	std::unique_ptr<Image> image = Image::fromFrameBuffer(buffer, Image::MapMode::ReadWrite);

	cv::Mat frame(streamConfig.size.height, streamConfig.size.width, CV_8UC4, image->data(0).data());
	cv::flip(frame, frame, -1);

	
	const auto json_tag = handleTags(frame);

	std::cout << json_tag<<'\n';

	client->publish(mqtt::make_message(TAGS_TOPIC, json_tag));

	request->reuse(Request::ReuseBuffers);
	camera->queueRequest(request);
}




static EventLoop loop;

static void requestComplete(Request *request)
{
	if (request->status() == Request::RequestCancelled)
		return;

	loop.callLater(std::bind(&processRequest, request));
}




class user_callback : public virtual mqtt::callback
{

	void connection_lost(const std::string& cause) override 
	{
		std::cout << "\nConnection lost\n";
		if (!cause.empty())
			std::cout << "\tcause: " << cause <<'\n';
		loop.exit();
	}
		
	
	void delivery_complete(mqtt::delivery_token_ptr tok) override
	{
		std::cout << "\n\t[Delivery complete for token: " << (tok ? tok->get_message_id() : -1) << "]\n";
	}

};


void signal_handler(int signal){
	loop.exit();
}


int main(int argc, char* argv[]) {
    
	std::signal(SIGINT, signal_handler);

	if(argc != 2){
		std::cout << "Numero di argomenti errato, inserisci indirizzo ip del broker MQTT";
		return -1;
	}

	std::string SERVER_ADDRESS { "mqtt://"};
	SERVER_ADDRESS += argv[1];
	SERVER_ADDRESS += ":1883";

	client = std::make_unique<mqtt::client>(SERVER_ADDRESS, CLIENT_ID);

	user_callback cb;
	client->set_callback(cb);

	mqtt::connect_options connOpts;
	connOpts.set_keep_alive_interval(20);
	connOpts.set_clean_session(true);
	connOpts.set_password(CLIENT_ID);
	connOpts.set_user_name(CLIENT_ID);

	try {
		std::cout << "Connecting...\n";
    	client->connect(connOpts);
    	std::cout << "Connected\n";
	}
	catch (const mqtt::exception& exc) {
		std::cout << exc.what();
		return -2;
	}


	std::unique_ptr<CameraManager> cm = std::make_unique<CameraManager>();
	cm->start();

	std::this_thread::sleep_for(1s);

	if (cm->cameras().empty()) {
		std::cout << "No cameras were identified on the system."<< std::endl;
		cm->stop();
		return 1;
	}

	camera = cm->cameras()[0];
	camera->acquire();

	std::unique_ptr<CameraConfiguration> config = camera->generateConfiguration( { StreamRole::Viewfinder} );

	StreamConfiguration &streamConfig = config->at(0);
	std::cout << "Default viewfinder configuration is: " << streamConfig.toString() << std::endl;
	config->validate();
	camera->configure(config.get());

	FrameBufferAllocator *allocator = new FrameBufferAllocator(camera);

	for (StreamConfiguration &cfg : *config)
		allocator->allocate(cfg.stream());


	Stream *stream = streamConfig.stream();
	const std::vector<std::unique_ptr<FrameBuffer>> &buffers = allocator->buffers(stream);
	std::vector<std::unique_ptr<Request>> requests;

	for (const std::unique_ptr<FrameBuffer>& buffer : buffers) {
		std::unique_ptr<Request> request = camera->createRequest();
		request->addBuffer(stream, buffer.get());
		requests.push_back(std::move(request));
	}

	camera->requestCompleted.connect(requestComplete);

	camera->start();

	for (std::unique_ptr<Request> &request : requests)
		camera->queueRequest(request.get());
        
    
    int ret = loop.exec();
    

	camera->stop();
	allocator->free(stream);
	delete allocator;
	camera->release();
	camera.reset();
	cm->stop();
	client->disconnect();

   
    return 0;
}



