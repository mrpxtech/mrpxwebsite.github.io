# SurgicalControl Android Integration Guide

This guide will help you integrate your Android app with the SurgicalControl authentication and usage tracking system.

## Quick Start

### 1. Base URL Configuration

```java
// Production
private static final String BASE_URL = "https://api.mrpxtech.com/api/";

// Development (if running locally)
private static final String BASE_URL = "http://localhost:3000/api/";
```

### 2. Required Dependencies

Add these to your `app/build.gradle`:

```gradle
dependencies {
    // Retrofit for API calls
    implementation 'com.squareup.retrofit2:retrofit:2.9.0'
    implementation 'com.squareup.retrofit2:converter-gson:2.9.0'
    implementation 'com.squareup.okhttp3:logging-interceptor:4.9.0'
    
    // Secure storage for tokens
    implementation 'androidx.security:security-crypto:1.1.0-alpha06'
    
    // JSON parsing
    implementation 'com.google.code.gson:gson:2.8.9'
}
```

## API Interface

Create an interface for your API calls:

```java
public interface SurgicalControlAPI {
    @POST("auth/signup")
    Call<AuthResponse> signup(@Body SignupRequest request);
    
    @POST("auth/login")
    Call<AuthResponse> login(@Body LoginRequest request);
    
    @GET("user/data")
    Call<UserData> getUserData(@Header("Authorization") String token);
    
    @POST("usage/start")
    Call<UsageResponse> startUsage(@Header("Authorization") String token);
    
    @POST("usage/stop")
    Call<UsageResponse> stopUsage(@Header("Authorization") String token, @Body StopUsageRequest request);
    
    @GET("user/usage/history")
    Call<UsageHistoryResponse> getUsageHistory(@Header("Authorization") String token, 
                                             @Query("limit") int limit, 
                                             @Query("offset") int offset);
    
    @GET("user/billing")
    Call<BillingResponse> getBilling(@Header("Authorization") String token,
                                   @Query("year") int year,
                                   @Query("month") int month);
    
    @PUT("user/profile")
    Call<ProfileResponse> updateProfile(@Header("Authorization") String token, @Body ProfileRequest request);
    
    @PUT("user/password")
    Call<PasswordResponse> changePassword(@Header("Authorization") String token, @Body PasswordRequest request);
    
    @GET("config")
    Call<ConfigResponse> getConfig();
}
```

## Data Models

### Request Models

```java
public class SignupRequest {
    private String fullName;
    private String email;
    private String password;
    private String plan;
    
    // Constructor, getters, setters
}

public class LoginRequest {
    private String email;
    private String password;
    
    // Constructor, getters, setters
}

public class StopUsageRequest {
    private int duration;
    
    // Constructor, getters, setters
}

public class ProfileRequest {
    private String fullName;
    private String plan;
    
    // Constructor, getters, setters
}

public class PasswordRequest {
    private String currentPassword;
    private String newPassword;
    
    // Constructor, getters, setters
}
```

### Response Models

```java
public class AuthResponse {
    private String message;
    private String token;
    private User user;
    
    // Getters, setters
}

public class User {
    private String id;
    private String fullName;
    private String email;
    private String plan;
    
    // Getters, setters
}

public class UserData {
    private String fullName;
    private String email;
    private String plan;
    private double balance;
    private int monthlyUsage;
    private double monthlyCost;
    
    // Getters, setters
}

public class UsageResponse {
    private String message;
    private String sessionId;
    private int duration;
    private double cost;
    
    // Getters, setters
}

public class UsageHistoryResponse {
    private List<UsageSession> sessions;
    private int total;
    private int limit;
    private int offset;
    
    // Getters, setters
}

public class UsageSession {
    private String id;
    private String startTime;
    private String endTime;
    private int duration;
    private double cost;
    
    // Getters, setters
}

public class BillingResponse {
    private int year;
    private int month;
    private int totalSeconds;
    private double totalCost;
    private String plan;
    private double ratePerSecond;
    
    // Getters, setters
}

public class ConfigResponse {
    private Map<String, Double> pricing;
    private Map<String, PlanInfo> plans;
    private String appVersion;
    private String apiVersion;
    
    // Getters, setters
}

public class PlanInfo {
    private String name;
    private double pricePerSecond;
    private List<String> features;
    
    // Getters, setters
}
```

## Implementation Example

### 1. Setup Retrofit

```java
public class ApiClient {
    private static final String BASE_URL = "https://api.mrpxtech.com/api/";
    private static Retrofit retrofit = null;
    
    public static Retrofit getClient() {
        if (retrofit == null) {
            OkHttpClient client = new OkHttpClient.Builder()
                .addInterceptor(new HttpLoggingInterceptor()
                    .setLevel(HttpLoggingInterceptor.Level.BODY))
                .build();
                
            retrofit = new Retrofit.Builder()
                .baseUrl(BASE_URL)
                .addConverterFactory(GsonConverterFactory.create())
                .client(client)
                .build();
        }
        return retrofit;
    }
}
```

### 2. Token Management

```java
public class TokenManager {
    private static final String PREF_NAME = "SurgicalControl";
    private static final String KEY_TOKEN = "auth_token";
    private static final String KEY_USER_DATA = "user_data";
    
    private SharedPreferences prefs;
    private SharedPreferences.Editor editor;
    
    public TokenManager(Context context) {
        prefs = context.getSharedPreferences(PREF_NAME, Context.MODE_PRIVATE);
        editor = prefs.edit();
    }
    
    public void saveToken(String token) {
        editor.putString(KEY_TOKEN, token);
        editor.apply();
    }
    
    public String getToken() {
        return prefs.getString(KEY_TOKEN, null);
    }
    
    public void saveUserData(User user) {
        Gson gson = new Gson();
        String userJson = gson.toJson(user);
        editor.putString(KEY_USER_DATA, userJson);
        editor.apply();
    }
    
    public User getUserData() {
        String userJson = prefs.getString(KEY_USER_DATA, null);
        if (userJson != null) {
            Gson gson = new Gson();
            return gson.fromJson(userJson, User.class);
        }
        return null;
    }
    
    public void clearData() {
        editor.clear();
        editor.apply();
    }
    
    public boolean isLoggedIn() {
        return getToken() != null;
    }
}
```

### 3. Login Implementation

```java
public class LoginActivity extends AppCompatActivity {
    private SurgicalControlAPI api;
    private TokenManager tokenManager;
    
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_login);
        
        api = ApiClient.getClient().create(SurgicalControlAPI.class);
        tokenManager = new TokenManager(this);
        
        // Check if already logged in
        if (tokenManager.isLoggedIn()) {
            startMainActivity();
            finish();
        }
    }
    
    private void login(String email, String password) {
        LoginRequest request = new LoginRequest(email, password);
        Call<AuthResponse> call = api.login(request);
        
        call.enqueue(new Callback<AuthResponse>() {
            @Override
            public void onResponse(Call<AuthResponse> call, Response<AuthResponse> response) {
                if (response.isSuccessful()) {
                    AuthResponse authResponse = response.body();
                    
                    // Save token and user data
                    tokenManager.saveToken(authResponse.getToken());
                    tokenManager.saveUserData(authResponse.getUser());
                    
                    // Navigate to main activity
                    startMainActivity();
                    finish();
                } else {
                    // Handle error
                    showError("Login failed: " + response.message());
                }
            }
            
            @Override
            public void onFailure(Call<AuthResponse> call, Throwable t) {
                showError("Network error: " + t.getMessage());
            }
        });
    }
    
    private void startMainActivity() {
        Intent intent = new Intent(this, MainActivity.class);
        startActivity(intent);
    }
    
    private void showError(String message) {
        Toast.makeText(this, message, Toast.LENGTH_LONG).show();
    }
}
```

### 4. Usage Tracking

```java
public class MainActivity extends AppCompatActivity {
    private SurgicalControlAPI api;
    private TokenManager tokenManager;
    private long sessionStartTime;
    private String sessionId;
    
    @Override
    protected void onResume() {
        super.onResume();
        startUsageTracking();
    }
    
    @Override
    protected void onPause() {
        super.onPause();
        stopUsageTracking();
    }
    
    private void startUsageTracking() {
        sessionStartTime = System.currentTimeMillis();
        String token = "Bearer " + tokenManager.getToken();
        
        Call<UsageResponse> call = api.startUsage(token);
        call.enqueue(new Callback<UsageResponse>() {
            @Override
            public void onResponse(Call<UsageResponse> call, Response<UsageResponse> response) {
                if (response.isSuccessful()) {
                    sessionId = response.body().getSessionId();
                }
            }
            
            @Override
            public void onFailure(Call<UsageResponse> call, Throwable t) {
                Log.e("UsageTracking", "Failed to start session", t);
            }
        });
    }
    
    private void stopUsageTracking() {
        if (sessionStartTime > 0) {
            long duration = (System.currentTimeMillis() - sessionStartTime) / 1000;
            String token = "Bearer " + tokenManager.getToken();
            
            StopUsageRequest request = new StopUsageRequest((int) duration);
            Call<UsageResponse> call = api.stopUsage(token, request);
            
            call.enqueue(new Callback<UsageResponse>() {
                @Override
                public void onResponse(Call<UsageResponse> call, Response<UsageResponse> response) {
                    if (response.isSuccessful()) {
                        UsageResponse usageResponse = response.body();
                        updateUsageDisplay(usageResponse);
                    }
                }
                
                @Override
                public void onFailure(Call<UsageResponse> call, Throwable t) {
                    Log.e("UsageTracking", "Failed to stop session", t);
                }
            });
        }
    }
    
    private void updateUsageDisplay(UsageResponse usageResponse) {
        // Update UI with usage data
        // This could be updating a TextView, showing a notification, etc.
    }
}
```

### 5. User Data Management

```java
public class ProfileActivity extends AppCompatActivity {
    private SurgicalControlAPI api;
    private TokenManager tokenManager;
    
    private void loadUserData() {
        String token = "Bearer " + tokenManager.getToken();
        Call<UserData> call = api.getUserData(token);
        
        call.enqueue(new Callback<UserData>() {
            @Override
            public void onResponse(Call<UserData> call, Response<UserData> response) {
                if (response.isSuccessful()) {
                    UserData userData = response.body();
                    displayUserData(userData);
                }
            }
            
            @Override
            public void onFailure(Call<UserData> call, Throwable t) {
                showError("Failed to load user data");
            }
        });
    }
    
    private void updateProfile(String fullName, String plan) {
        String token = "Bearer " + tokenManager.getToken();
        ProfileRequest request = new ProfileRequest(fullName, plan);
        
        Call<ProfileResponse> call = api.updateProfile(token, request);
        call.enqueue(new Callback<ProfileResponse>() {
            @Override
            public void onResponse(Call<ProfileResponse> call, Response<ProfileResponse> response) {
                if (response.isSuccessful()) {
                    showSuccess("Profile updated successfully");
                    loadUserData(); // Refresh data
                } else {
                    showError("Failed to update profile");
                }
            }
            
            @Override
            public void onFailure(Call<ProfileResponse> call, Throwable t) {
                showError("Network error");
            }
        });
    }
}
```

## Error Handling

### Network Error Handling

```java
public class NetworkUtils {
    public static boolean isNetworkAvailable(Context context) {
        ConnectivityManager connectivityManager = (ConnectivityManager) 
            context.getSystemService(Context.CONNECTIVITY_SERVICE);
        
        if (connectivityManager != null) {
            NetworkCapabilities capabilities = connectivityManager
                .getNetworkCapabilities(connectivityManager.getActiveNetwork());
            
            return capabilities != null && (
                capabilities.hasTransport(NetworkCapabilities.TRANSPORT_WIFI) ||
                capabilities.hasTransport(NetworkCapabilities.TRANSPORT_CELLULAR)
            );
        }
        return false;
    }
}
```

### Token Expiration Handling

```java
public class TokenInterceptor implements Interceptor {
    private TokenManager tokenManager;
    private Context context;
    
    public TokenInterceptor(Context context) {
        this.context = context;
        this.tokenManager = new TokenManager(context);
    }
    
    @Override
    public Response intercept(Chain chain) throws IOException {
        Request original = chain.request();
        
        // Add token to request
        String token = tokenManager.getToken();
        if (token != null) {
            Request request = original.newBuilder()
                .header("Authorization", "Bearer " + token)
                .build();
            
            Response response = chain.proceed(request);
            
            // Handle token expiration
            if (response.code() == 403) {
                // Token expired, redirect to login
                tokenManager.clearData();
                Intent intent = new Intent(context, LoginActivity.class);
                intent.addFlags(Intent.FLAG_ACTIVITY_NEW_TASK | Intent.FLAG_ACTIVITY_CLEAR_TASK);
                context.startActivity(intent);
            }
            
            return response;
        }
        
        return chain.proceed(original);
    }
}
```

## Security Best Practices

### 1. Secure Token Storage

```java
public class SecureTokenManager {
    private EncryptedSharedPreferences prefs;
    
    public SecureTokenManager(Context context) throws GeneralSecurityException, IOException {
        String masterKeyAlias = MasterKeys.getOrCreate(MasterKeys.AES256_GCM_SPEC);
        
        prefs = EncryptedSharedPreferences.create(
            "secure_prefs",
            masterKeyAlias,
            context,
            EncryptedSharedPreferences.PrefKeyEncryptionScheme.AES256_SIV,
            EncryptedSharedPreferences.PrefValueEncryptionScheme.AES256_GCM
        );
    }
    
    public void saveToken(String token) {
        prefs.edit().putString("auth_token", token).apply();
    }
    
    public String getToken() {
        return prefs.getString("auth_token", null);
    }
}
```

### 2. Certificate Pinning (Optional)

```java
public class CertificatePinning {
    public static OkHttpClient.Builder addCertificatePinning(OkHttpClient.Builder builder) {
        CertificatePinner certificatePinner = new CertificatePinner.Builder()
            .add("api.mrpxtech.com", "sha256/AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA=")
            .build();
        
        return builder.certificatePinner(certificatePinner);
    }
}
```

## Testing

### 1. Unit Tests

```java
@Test
public void testLoginSuccess() {
    // Mock API response
    AuthResponse mockResponse = new AuthResponse();
    mockResponse.setToken("test_token");
    mockResponse.setMessage("Login successful");
    
    // Test login logic
    // ...
}
```

### 2. Integration Tests

```java
@Test
public void testApiIntegration() {
    // Test actual API calls
    SurgicalControlAPI api = ApiClient.getClient().create(SurgicalControlAPI.class);
    
    LoginRequest request = new LoginRequest("test@example.com", "password");
    Call<AuthResponse> call = api.login(request);
    
    // Verify response
    // ...
}
```

## Deployment Checklist

- [ ] Update BASE_URL to production endpoint
- [ ] Implement proper error handling
- [ ] Add network connectivity checks
- [ ] Test token expiration handling
- [ ] Verify secure token storage
- [ ] Test usage tracking accuracy
- [ ] Implement offline handling
- [ ] Add proper logging
- [ ] Test on different Android versions
- [ ] Verify API rate limiting compliance

## Support

For API support and questions:
- Email: support@surgicalcontrol.com
- Documentation: https://mrpxtech.com/api-docs.html
- GitHub Issues: [Your Repository URL]

## Version History

- v1.0.0 - Initial release with basic authentication and usage tracking
- v1.1.0 - Added billing and profile management endpoints
- v1.2.0 - Enhanced security with certificate pinning support 