from flask import Flask, jsonify, request

# Import shared globals from main.py.
# (This creates a circular import, but if used only inside functions it will work.)
from main import available_params, streaming_history, history_by_type, alink_sent_history, history_lock, params_lock

def run_flask():
    app = Flask(__name__)

    @app.route("/parameters", methods=["GET"])
    def get_parameters():
        with params_lock:
            return jsonify(available_params)

    @app.route("/parameters/<param_id>", methods=["GET"])
    def get_parameter(param_id):
        with params_lock:
            param = available_params.get(param_id)
        if param is None:
            return jsonify({"error": "Parameter not found"}), 404
        return jsonify({param_id: param})

    @app.route("/parameters/<param_id>", methods=["POST"])
    def set_parameter(param_id):
        if not request.json or "value" not in request.json:
            return jsonify({"error": "Missing new value in request"}), 400
        new_value = request.json["value"]
        with params_lock:
            param = available_params.get(param_id)
            if param is None:
                return jsonify({"error": "Parameter not found"}), 404
            param["value"] = new_value
            available_params[param_id] = param
        from main import dest_mav_conn
        if dest_mav_conn is None:
            return jsonify({"error": "No MAVLink connection available"}), 500
        try:
            dest_mav_conn.mav.param_set_send(1, 1, param_id.encode('ascii'), float(new_value), param["type"])
        except Exception as e:
            return jsonify({"error": f"Failed to send PARAM_SET: {e}"}), 500
        return jsonify({"status": "Parameter updated", param_id: param})

    @app.route("/stream", methods=["GET"])
    def get_stream():
        msg_type = request.args.get("type")
        with history_lock:
            if msg_type:
                return jsonify(history_by_type.get(msg_type, []))
            else:
                return jsonify(streaming_history)

    @app.route("/alink_stream", methods=["GET"])
    def get_alink_stream():
        with history_lock:
            return jsonify(alink_sent_history)

    # Run the Flask app.
    app.run(host="0.0.0.0", port=5000, threaded=True, use_reloader=False)
