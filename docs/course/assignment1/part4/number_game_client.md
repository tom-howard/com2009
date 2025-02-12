---  
title: Creating a Python Service Client
---

Copy **all** the code below into your `number_game_client.py` file and then **review the annotations** to understand how it all works.

```python title="number_game_client.py"
--8<-- "code_templates/number_game_client.py"
```

1. Nothing above this should be new to you. 
    
    Here however, we're importing a standard Python module called `argparse`, which we'll use to build a *command-line interface* for our node.

2. Creating a Service *Client* is done using the `create_client()` class method, providing the name of the service that we want to call (`srv_name`), and specifying the interface type used by it (`srv_type`). 

    `srv_type` and `srv_name` **must** match the definition in the server, in order to be able to communicate and send requests to it. 

3. Here we're building a simple Command-line Interface (CLI) for the node, using `argparse`.

4. We add an argument here called "guess", which will be used to pass our guesses for the number game from the command line, into this node.
    
    The node itself can then access the value that we've passed to it (an `int`) and use this to construct a service request.

    We've assigned a default value of `0` here, for cases where we don't pass a guess from the CLI.

5. Here we're adding a *second* Command-line Argument (CLA) called "cheat".

    `#!py action="store_true"` ensures that if we don't specify this argument as a CLA then the value will be set to `False`. If we do pass in this argument then the value will be `True`. 

    (You'll see how this all works shortly, when we actually run the node.)

6. Here we "parse" the arguments that have been passed to the node from the CLI, so that we can access them from `self.args`.

7. We use a `while` loop here to halt the execution of the code at this point and wait for the service to become available (if it isn't already). 

    We can't send a request to a service that isn't actually running!

8. In this class method we construct a service **request**, based on the values that have been passed via the CLI.
    
    (This method is called in the `main()` function below.)

    We know what the **request** attributes are called, because we defined them in the `MyNumberGame.srv` file, and we can also use `ros2 interface show` to recall them at any point:

    ``` { .txt .no-copy }
    $ ros2 interface show part4_services/srv/MyNumberGame
    
    int32 guess
    bool cheat
    ---
    int32 num_guesses
    string hint
    bool correct
    ```

    `#!py call_async(request)` then actually sends this request to the server.

9. Here we're grabbing the values passed via the CLI and printing them as a log message to the terminal, in order to verify exactly what request will be sent.

10. We then call our client's `send_request()` class method, supplying the `guess` and `cheat` values from the CLI to this too, in order for the request to be constructed accordingly, and sent to the server. 

    The output of this function is the output of the `call_async(request)` call, which we assign to a variable called `future`.

11. We use the `rclpy.spin_until_future_complete()` method here, which (as the name suggests) will allow our node (`client`) to spin *only* until our service request (`future`) has completed. 

12. Once we've reached this point then the service has completed and returned its **Response**. 
    
    We obtain the response from our `future` object so that we can read its values...

13. To finish off, we construct another log message to contain all the values returned by the Server (i.e. the **Response**). 
    
    We know what these attributes are called, because we defined them in the `MyNumberGame.srv` file, which we can recall at any point using `ros2 interface show`:

    ``` { .txt .no-copy }
    $ ros2 interface show part4_services/srv/MyNumberGame
    
    int32 guess
    bool cheat
    ---
    int32 num_guesses
    string hint
    bool correct
    ```


<p align="center">
  <a href="../../part4#ex_srv_cli_ret">&#8592; Back to Part 4</a>
</p>