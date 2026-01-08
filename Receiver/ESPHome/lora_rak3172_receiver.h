#pragma once

#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

namespace esphome
{
  namespace lora_rak3172_receiver
  {

    static const char *const TAG = "lora_rak3172_receiver";

    class LoRaRAK3172Receiver : public Component, public uart::UARTDevice
    {
    public:
      // Safety cap to prevent unbounded growth if line terminators are missing.
      static constexpr size_t MAX_RX_BUFFER_LEN = 256;

      void set_lora_frequency(uint32_t frequency) { this->frequency_ = frequency; }
      void set_lora_sf(int sf) { this->sf_ = sf; }
      void set_lora_bw(int bw) { this->bw_ = bw; }
      void set_lora_cr(int cr) { this->cr_ = cr; }
      void set_lora_preamble(int preamble) { this->preamble_ = preamble; }
      void set_lora_tx_power(int tx_power) { this->tx_power_ = tx_power; }

      void set_out_temp(sensor::Sensor *s) { this->out_temp_ = s; }
      void set_out_hum(sensor::Sensor *s) { this->out_hum_ = s; }
      void set_out_batt(sensor::Sensor *s) { this->out_batt_ = s; }

      void set_int_temp(sensor::Sensor *s) { this->int_temp_ = s; }
      void set_int_hum(sensor::Sensor *s) { this->int_hum_ = s; }
      void set_int_batt(sensor::Sensor *s) { this->int_batt_ = s; }

      void set_fan_state(binary_sensor::BinarySensor *s) { this->fan_state_ = s; }

      void set_rssi(sensor::Sensor *s) { this->rssi_ = s; }
      void set_snr(sensor::Sensor *s) { this->snr_ = s; }

      void set_last_raw_payload(text_sensor::TextSensor *s) { this->last_raw_payload_ = s; }
      void set_last_crc_status(text_sensor::TextSensor *s) { this->last_crc_status_ = s; }

      void setup() override
      {
        ESP_LOGI(TAG, "Initializing RAK3172 P2P receiver...");
        ESP_LOGI(TAG, "Config freq=%lu sf=%d bw=%d cr=%d preamble=%d tx_power=%d",
                 static_cast<unsigned long>(this->frequency_),
                 this->sf_,
                 this->bw_,
                 this->cr_,
                 this->preamble_,
                 this->tx_power_);
        // Basic AT handshake and mode selection.
        this->send_cmd_("AT");
        this->read_lines_for_(1000);
        this->send_cmd_("AT+VER?");
        this->read_lines_for_(1000);
        this->send_cmd_("AT+NWM=0");
        this->read_lines_for_(500);

        // Configure P2P radio parameters.
        char cfg[64];
        this->build_p2p_cmd_(cfg, sizeof(cfg));
        ESP_LOGI(TAG, "TX: %s", cfg);
        this->send_cmd_(cfg);
        bool p2p_ok = this->read_lines_for_(200);
        ESP_LOGI(TAG, "P2P response: %s", p2p_ok ? "OK" : "NO_OK");
        // Enter continuous RX mode.
        this->start_receive_();
        ESP_LOGI(TAG, "RAK3172 receiver ready");
      }

      void loop() override
      {
        uint32_t now = millis();
        while (this->available())
        {
          int c = this->read();
          if (c < 0)
          {
            break;
          }
          if (c == '\n')
          {
            std::string line = this->rx_buffer_;
            this->rx_buffer_.clear();
            this->handle_line_(line);
          }
          else if (c != '\r')
          {
            // Drop the current line if it exceeds the cap to avoid heap growth.
            if (this->rx_buffer_.size() < MAX_RX_BUFFER_LEN)
            {
              this->rx_buffer_.push_back(static_cast<char>(c));
            }
            else
            {
              ESP_LOGW(TAG, "RX buffer overflow, clearing");
              this->rx_buffer_.clear();
            }
          }
        }

        (void)now;
      }

    protected:
      void send_cmd_(const std::string &cmd)
      {
        ESP_LOGD(TAG, "TX: %s", cmd.c_str());
        this->write_str(cmd.c_str());
        this->write_str("\r\n");
        this->flush();
      }

      void start_receive_()
      {
        ESP_LOGD(TAG, "Starting PRECV");
        // 65534 keeps the module in continuous receive per vendor docs.
        this->send_cmd_("AT+PRECV=65534");
      }

      void build_p2p_cmd_(char *buf, size_t buf_len) const
      {
        // Format: AT+P2P=Frequency:SF:BW:CR:Preamble:TXPower
        snprintf(buf, buf_len, "AT+P2P=%lu:%d:%d:%d:%d:%d",
                 static_cast<unsigned long>(this->frequency_),
                 this->sf_,
                 this->bw_,
                 this->cr_,
                 this->preamble_,
                 this->tx_power_);
      }

      bool read_lines_for_(uint32_t timeout_ms)
      {
        bool saw_ok = false;
        uint32_t start = millis();
        while (millis() - start < timeout_ms)
        {
          while (this->available())
          {
            int c = this->read();
            if (c < 0)
            {
              break;
            }
            if (c == '\n')
            {
              std::string line = this->rx_buffer_;
              this->rx_buffer_.clear();
              line = trim_(line);
              if (!line.empty())
              {
                ESP_LOGI(TAG, "RX(test): %s", line.c_str());
                if (line == "OK")
                {
                  saw_ok = true;
                }
              }
            }
            else if (c != '\r')
            {
              // Drop the current line if it exceeds the cap to avoid heap growth.
              if (this->rx_buffer_.size() < MAX_RX_BUFFER_LEN)
              {
                this->rx_buffer_.push_back(static_cast<char>(c));
              }
              else
              {
                ESP_LOGW(TAG, "RX buffer overflow, clearing");
                this->rx_buffer_.clear();
              }
            }
          }
          delay(10);
        }
        return saw_ok;
      }

      static std::string trim_(const std::string &s)
      {
        size_t start = 0;
        while (start < s.size() && std::isspace(static_cast<unsigned char>(s[start])))
        {
          start++;
        }
        size_t end = s.size();
        while (end > start && std::isspace(static_cast<unsigned char>(s[end - 1])))
        {
          end--;
        }
        return s.substr(start, end - start);
      }

      static bool is_hex_(const std::string &s)
      {
        if (s.empty() || (s.size() % 2) != 0)
        {
          return false;
        }
        for (char c : s)
        {
          if (!std::isxdigit(static_cast<unsigned char>(c)))
          {
            return false;
          }
        }
        return true;
      }

      static std::string hex_to_ascii_(const std::string &hex)
      {
        std::string out;
        out.reserve(hex.size() / 2);
        for (size_t i = 0; i < hex.size(); i += 2)
        {
          char tmp[3] = {hex[i], hex[i + 1], 0};
          char b = static_cast<char>(strtoul(tmp, nullptr, 16));
          out.push_back(b);
        }
        return out;
      }

      static bool parse_int_token_(const std::string &token, int *out)
      {
        if (token.empty())
        {
          return false;
        }
        size_t i = 0;
        if (token[0] == '+' || token[0] == '-')
        {
          i = 1;
        }
        if (i >= token.size())
        {
          return false;
        }
        for (; i < token.size(); i++)
        {
          if (!std::isdigit(static_cast<unsigned char>(token[i])))
          {
            return false;
          }
        }
        *out = atoi(token.c_str());
        return true;
      }

      static bool parse_rssi_snr_(const std::string &line, int *rssi, int *snr)
      {
        if (line.find("RXP2P") == std::string::npos)
        {
          return false;
        }

        std::vector<int> values;
        std::string token;
        token.reserve(line.size());
        for (char c : line)
        {
          if (c == ',' || c == ':')
          {
            std::string t = trim_(token);
            token.clear();
            if (t.empty())
            {
              continue;
            }
            if (t.find("RXP2P") != std::string::npos || t.find("EVT") != std::string::npos)
            {
              continue;
            }
            int v = 0;
            if (parse_int_token_(t, &v))
            {
              values.push_back(v);
            }
            continue;
          }
          token.push_back(c);
        }
        if (!token.empty())
        {
          std::string t = trim_(token);
          int v = 0;
          if (parse_int_token_(t, &v))
          {
            values.push_back(v);
          }
        }

        if (values.empty())
        {
          return false;
        }
        for (size_t i = 0; i + 1 < values.size(); i++)
        {
          if (values[i] < 0)
          {
            *rssi = values[i];
            *snr = values[i + 1];
            return true;
          }
        }
        if (values.size() >= 2)
        {
          *rssi = values[0];
          *snr = values[1];
          return true;
        }
        return false;
      }

      void handle_line_(const std::string &line)
      {
        std::string s = trim_(line);
        ESP_LOGV(TAG, "RX: %s", s.c_str());
        if (s.empty() || s == "OK" || s == "ERROR")
        {
          return;
        }

        int rssi = 0;
        int snr = 0;
        if (parse_rssi_snr_(s, &rssi, &snr))
        {
          if (this->rssi_ != nullptr)
          {
            this->rssi_->publish_state(rssi);
          }
          if (this->snr_ != nullptr)
          {
            this->snr_->publish_state(snr);
          }
        }

        // Try to extract payload portion from common RAK response formats.
        std::string payload;
        size_t colon = s.rfind(':');
        if (colon != std::string::npos && colon + 1 < s.size())
        {
          payload = s.substr(colon + 1);
        }
        else
        {
          size_t comma = s.rfind(',');
          if (comma != std::string::npos && comma + 1 < s.size())
          {
            payload = s.substr(comma + 1);
          }
        }

        payload = trim_(payload);
        ESP_LOGV(TAG, "Payload raw: %s", payload.c_str());
        if (payload.size() >= 2 && payload.front() == '"' && payload.back() == '"')
        {
          payload = payload.substr(1, payload.size() - 2);
        }
        if (payload.empty())
        {
          return;
        }

        // RAK may deliver payload as hex, convert to ASCII if needed.
        if (is_hex_(payload))
        {
          payload = hex_to_ascii_(payload);
          ESP_LOGV(TAG, "Payload hex->ascii: %s", payload.c_str());
        }

        if (this->last_raw_payload_ != nullptr)
        {
          this->last_raw_payload_->publish_state(payload);
        }

        if (!this->parse_payload_(payload))
        {
          ESP_LOGW(TAG, "Invalid payload: %s", payload.c_str());
        }
        else
        {
          ESP_LOGD(TAG, "Parsed payload: %s", payload.c_str());
        }
      }

      bool parse_payload_(const std::string &payload)
      {
        // Expected payload: 19 bytes data + 4 hex CRC16 (total 23 chars).
        if (payload.size() != 23)
        {
          ESP_LOGW(TAG, "Length error: %u bytes (expected 23)", static_cast<unsigned>(payload.size()));
          if (this->last_crc_status_ != nullptr)
          {
            this->last_crc_status_->publish_state("length_error");
          }
          return false;
        }

        const std::string data_part = payload.substr(0, payload.size() - 4);
        const std::string crc_str = payload.substr(payload.size() - 4);
        if (data_part.size() != 19)
        {
          ESP_LOGW(TAG, "Data part length error: %u bytes (expected 19)",
                   static_cast<unsigned>(data_part.size()));
          if (this->last_crc_status_ != nullptr)
          {
            this->last_crc_status_->publish_state("length_error");
          }
          return false;
        }

        uint16_t recv_crc = static_cast<uint16_t>(strtoul(crc_str.c_str(), nullptr, 16));
        uint16_t calc_crc = compute_crc16_(reinterpret_cast<const uint8_t *>(data_part.c_str()),
                                           data_part.size());
        if (recv_crc != calc_crc)
        {
          ESP_LOGW(TAG, "CRC mismatch recv=0x%04X calc=0x%04X", recv_crc, calc_crc);
          if (this->last_crc_status_ != nullptr)
          {
            this->last_crc_status_->publish_state("crc_mismatch");
          }
          return false;
        }
        ESP_LOGD(TAG, "CRC OK 0x%04X", calc_crc);

        if (this->last_crc_status_ != nullptr)
        {
          this->last_crc_status_->publish_state("ok");
        }

        bool fan_on = (data_part.back() == '1');
        if (this->fan_state_ != nullptr)
        {
          this->fan_state_->publish_state(fan_on);
        }

        // Each sensor block is 1+3+3+2 chars: ID, temp*10, hum*10, batt.
        int idx = 0;
        for (int i = 0; i < 2; i++)
        {
          if (idx + 9 > static_cast<int>(data_part.size() - 1))
          {
            ESP_LOGW(TAG, "Parse overflow at idx=%d", idx);
            return false;
          }
          int sensor_id = data_part.at(idx++) - '0';
          int t = atoi(data_part.substr(idx, 3).c_str());
          idx += 3;
          int h = atoi(data_part.substr(idx, 3).c_str());
          idx += 3;
          int b = atoi(data_part.substr(idx, 2).c_str());
          idx += 2;

          float temperature = t / 10.0f;
          float humidity = h / 10.0f;
          ESP_LOGD(TAG, "Sensor %d -> T=%.1fC H=%.1f%% B=%d%% Fan=%s",
                   sensor_id, temperature, humidity, b, fan_on ? "ON" : "OFF");

          if (sensor_id == 1)
          {
            if (this->out_temp_ != nullptr)
              this->out_temp_->publish_state(temperature);
            if (this->out_hum_ != nullptr)
              this->out_hum_->publish_state(humidity);
            if (this->out_batt_ != nullptr)
              this->out_batt_->publish_state(b);
          }
          else if (sensor_id == 2)
          {
            if (this->int_temp_ != nullptr)
              this->int_temp_->publish_state(temperature);
            if (this->int_hum_ != nullptr)
              this->int_hum_->publish_state(humidity);
            if (this->int_batt_ != nullptr)
              this->int_batt_->publish_state(b);
          }
        }

        return true;
      }

      static uint16_t compute_crc16_(const uint8_t *data, size_t len)
      {
        uint16_t crc = 0xFFFF;
        while (len--)
        {
          crc ^= static_cast<uint16_t>(*data++) << 8;
          for (uint8_t i = 0; i < 8; i++)
          {
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<uint16_t>(crc << 1);
          }
        }
        return crc;
      }

      uint32_t frequency_{868000000};
      int sf_{10};
      int bw_{1};
      int cr_{1};
      int preamble_{8};
      int tx_power_{22};

      sensor::Sensor *out_temp_{nullptr};
      sensor::Sensor *out_hum_{nullptr};
      sensor::Sensor *out_batt_{nullptr};
      sensor::Sensor *int_temp_{nullptr};
      sensor::Sensor *int_hum_{nullptr};
      sensor::Sensor *int_batt_{nullptr};
      sensor::Sensor *rssi_{nullptr};
      sensor::Sensor *snr_{nullptr};
      binary_sensor::BinarySensor *fan_state_{nullptr};
      text_sensor::TextSensor *last_raw_payload_{nullptr};
      text_sensor::TextSensor *last_crc_status_{nullptr};

      std::string rx_buffer_;
    };

  } // namespace lora_rak3172_receiver
} // namespace esphome
