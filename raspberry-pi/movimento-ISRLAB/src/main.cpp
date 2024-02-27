#include <string>
#include <csignal>
#include <chrono>
#include <thread>

#include <ecl/console.hpp>
#include <ecl/geometry.hpp>
#include <ecl/linear_algebra.hpp>
#include <ecl/time.hpp>
#include <ecl/sigslots.hpp>
#include <ecl/exceptions.hpp>

#include "kobuki_core/kobuki.hpp"
#include "kobuki_core/modules/battery.hpp"

#include <mqtt/client.h>

using namespace std::chrono_literals;



enum Command{
    LEFT,
    FORWARDLEFT,
    FORWARD,
    FORWARDRIGHT,
    RIGHT,
    SLOW_LEFT,
    SLOW_RIGHT,
    STOP = -1
};

class KobukiManager
{
public:

  KobukiManager(const std::string &device, mqtt::client& client);
  ~KobukiManager();


  const ecl::linear_algebra::Vector3d &getPose() { return pose; };
  bool isShutdown() { return kobuki.isShutdown(); }
  void processStreamData();

private:
  double vx, wz;
  ecl::linear_algebra::Vector3d pose;
  kobuki::Kobuki kobuki;
  ecl::Slot<> slot_stream_data;
  mqtt::client& client;

public:

  void doAction(const std::string &message);

};


KobukiManager::KobukiManager(const std::string &device, mqtt::client& client) : client{client}, slot_stream_data(&KobukiManager::processStreamData, *this){
  
  vx = 0.0;
  wz = 0.0;


  kobuki::Parameters parameters;
  parameters.sigslots_namespace = "/kobuki";
  parameters.device_port = device;
  parameters.enable_acceleration_limiter = true;

  kobuki.init(parameters);
  kobuki.enable();
  slot_stream_data.connect("/kobuki/stream_data");

}

KobukiManager::~KobukiManager()
{
  kobuki.setBaseControl(0, 0); // linear_velocity, angular_velocity in (m/s), (rad/s)
  kobuki.disable();
}


void KobukiManager::doAction(const std::string &message)
{ 

  Command command = static_cast<Command>(std::stoi(message));

  switch (command)
  {
    case FORWARD:
      std::cout << "FORWARD\n";
      vx = 0.1;
      wz = 0;
      break;
    case FORWARDLEFT:
      std::cout << "FORWARDLEFT\n";
      vx = 0.1;
      wz = 0.2;
      break;
    case FORWARDRIGHT:
      std::cout << "FORWARDRIGHT\n";
      vx = 0.1;
      wz = -0.2;
      break;
    case LEFT:
      std::cout << "LEFT\n";
      vx = 0;
      wz = 0.3;
      break;
    case RIGHT:
      std::cout << "RIGHT\n";
      vx = 0;
      wz = -0.3;
      break;
    case SLOW_LEFT:
      std::cout << "SLOW_LEFT\n";
      vx = 0;
      wz = 0.2;
      break;
    case SLOW_RIGHT:
      std::cout << "SLOW_RIGHT\n";
      vx = 0;
      wz = -0.2;
      break;
    case STOP:
      std::cout << "STOP\n";
      vx = 0;
      wz = 0;
      break;
    default:
      std::cout << "Azione non supportata: " << message <<'\n';
      break;
  }
  
}

void KobukiManager::processStreamData() {
  ecl::linear_algebra::Vector3d pose_update;
  ecl::linear_algebra::Vector3d pose_update_rates;
  kobuki.updateOdometry(pose_update, pose_update_rates);
  ecl::concatenate_poses(pose, pose_update);
  kobuki.setBaseControl(vx, wz);

}

volatile bool running = true;

class user_callback : public virtual mqtt::callback
{

      void connection_lost(const std::string &cause) override
      {
        std::cout << "\nConnection lost\n";
        if (!cause.empty())
          std::cout << "\tcause: " << cause <<'\n';
        running = false;
      }

      void delivery_complete(mqtt::delivery_token_ptr tok) override
      {
        std::cout << "\n\t[Delivery complete for token: " << (tok ? tok->get_message_id() : -1) << "]\n";
      }

    KobukiManager km;

    public:

      user_callback(mqtt::client& client): km("/dev/kobuki", client) {};

      void message_arrived(mqtt::const_message_ptr msg) override
      {
        std::cout << "Contenuto del messaggio: ";

        km.doAction(msg->get_payload_str());
      }
};

int main(int argc, char* argv[])
{

  //ecl::Sleep sleep(1);

  if(argc != 2){
		std::cout << "Numero di argomenti errato, inserisci indirizzo ip del broker MQTT\n";
		return -1;
	}

	std::string SERVER_ADDRESS { "mqtt://"};
	SERVER_ADDRESS += argv[1];
	SERVER_ADDRESS += ":1883";

  const std::string CLIENT_ID{"action_module"};
  const std::string TOPIC{"/actions"};

  // Creo il client con le credenziali corrette
  mqtt::client client(SERVER_ADDRESS, CLIENT_ID);

  user_callback cb{client};
  client.set_callback(cb);

  mqtt::connect_options connOpts;
  connOpts.set_keep_alive_interval(20);
  connOpts.set_clean_session(true);
  connOpts.set_user_name(CLIENT_ID);
  connOpts.set_password(CLIENT_ID);

  std::cout << "...OK\n";

  try
  {
    std::cout << "Connecting...\n";
    client.connect(connOpts);
    std::cout << "Connected\n";

    mqtt::subscribe_options subOpts(1);

    mqtt::properties subProps;


    client.subscribe(TOPIC, subOpts, subProps);

    // Loop personalizzato per mantenere il client in ascolto
    while (running)
    {
      // Controllo se ci sono nuovi messaggi
      client.start_consuming();

      // Attendo un po' prima di controllare nuovamente
      std::this_thread::sleep_for(100ms);
    }
  }
  catch (const mqtt::exception &exc)
  {
    std::cout << exc.what();
    return -2;
  }

  return 0;
}
