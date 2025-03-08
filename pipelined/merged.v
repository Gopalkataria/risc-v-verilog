module delete_files;

  initial begin
    // This will execute a shell command to delete all files in the folder
    // Be very careful when using "rm -rf" as it can delete files permanently
    $system("echo hello ")
    $display("All files in the folder have been deleted.");
  end

endmodule