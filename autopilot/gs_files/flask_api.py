from flask import Flask, jsonify, request, send_from_directory

def run_flask(shared_state):
    app = Flask(__name__)

    @app.route("/")
    def index():
        return send_from_directory("static", "index.html")

    @app.route("/parameters", methods=["GET"])
    def get_parameters():
        with shared_state.params_lock:
            return jsonify(shared_state.available_params)

    @app.route("/parameters/<param_id>", methods=["GET"])
    def get_parameter(param_id):
        with shared_state.params_lock:
            param = shared_state.available_params.get(param_id)
        if param is None:
            return jsonify({"error": "Parameter not found"}), 404
        return jsonify({param_id: param})

    @app.route("/parameters/<param_id>", methods=["POST"])
    def set_parameter(param_id):
        if not request.json or "value" not in request.json:
            return jsonify({"error": "Missing new value in request"}), 400
        new_value = request.json["value"]
        with shared_state.params_lock:
            param = shared_state.available_params.get(param_id)
            if param is None:
                return jsonify({"error": "Parameter not found"}), 404
            param["value"] = new_value
            shared_state.available_params[param_id] = param
        if not shared_state.dest_mav_conn:
            return jsonify({"error": "No MAVLink connection available"}), 500
        try:
            shared_state.dest_mav_conn.mav.param_set_send(1, 1, param_id.encode('ascii'), float(new_value), param["type"])
        except Exception as e:
            return jsonify({"error": f"Failed to send PARAM_SET: {e}"}), 500
        return jsonify({"status": "Parameter updated", param_id: param})

    @app.route("/stream", methods=["GET"])
    def get_stream():
        msg_type = request.args.get("type")
        with shared_state.history_lock:
            if msg_type:
                return jsonify(shared_state.history_by_type.get(msg_type, []))
            else:
                return jsonify(shared_state.streaming_history)

    @app.route("/alink_stream", methods=["GET"])
    def get_alink_stream():
        with shared_state.history_lock:
            return jsonify(shared_state.alink_sent_history)

    # New endpoint to send STATUSTEXT messages
    @app.route("/send_statustext", methods=["POST"])
    def send_statustext():
        if not request.json or "message" not in request.json:
            return jsonify({"error": "Missing 'message' field"}), 400
        text = request.json["message"]
        if not shared_state.dest_mav_conn:
            return jsonify({"error": "No MAVLink connection available"}), 500
        try:
            # Encode the message as ASCII to satisfy pymavlink's requirements.
            shared_state.dest_mav_conn.mav.statustext_send(6, text.encode('ascii'))
        except Exception as e:
            return jsonify({"error": f"Failed to send STATUSTEXT: {e}"}), 500
        return jsonify({"status": "STATUSTEXT sent", "message": text})
    
    # Run the Flask app.
    app.run(host="0.0.0.0", port=5000, threaded=True, use_reloader=False)
